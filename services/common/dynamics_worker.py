# -*- coding: utf-8 -*-
"""
dynamics_worker.py
==================
多体动力学计算独立 Worker 进程（独立后端模块 #5）

职责：
- 隔离高 CPU 的多体动力学（stability_analysis.StabilityAnalyzer）
- 使用 multiprocessing.Process + multiprocessing.Queue 实现主进程/Worker通信
- 支持批量任务提交，异步返回结果，避免阻塞事件循环
- 可独立运行：python services/common/dynamics_worker.py 启动独立Worker进程
- 可作为库导入：from common.dynamics_worker import DynamicsWorkerClient
"""
import os
import sys
import time
import math
import traceback
import multiprocessing as mp
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass, asdict
from queue import Empty

# 在子进程中避免 __main__ 导入问题
if __name__ != '__main__':
    from .stability_analysis import (
        VehicleDynamicsParams, CargoConfig, StabilityAnalyzer
    )
    from .extended_models import MultiVehicleSteeringModel, RoadSurfaceModel

MP_SPAWN_CTX = mp.get_context('spawn')

# ---- Worker 端消息 ----
MSG_TYPE_TASK = 'TASK'
MSG_TYPE_RESULT = 'RESULT'
MSG_TYPE_STOP = 'STOP'
MSG_TYPE_PING = 'PING'
MSG_TYPE_PONG = 'PONG'


@dataclass
class DynamicsTask:
    """动力学计算任务"""
    task_id: str
    vehicle_type: str
    pole_angle_deg: float
    speed_mps: float
    roll_angle_deg: float = 0.0
    friction_coeff: float = 0.7
    road_type: str = 'dirt_road'
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0
    cargo_shift_stiffness: float = 20000.0
    cargo_shift_damping: float = 800.0
    cargo_shift_dynamics: bool = False


class _DynamicsWorkerServer:
    """Worker 进程内执行体（spawn子进程，纯计算，无IO）"""

    def __init__(self, request_q: mp.Queue, response_q: mp.Queue):
        self._req = request_q
        self._resp = response_q
        self._steering = None
        self._road = None

    def _lazy_init(self):
        if self._steering is None:
            # 仅在首次任务时初始化，加快子进程启动
            self._steering = MultiVehicleSteeringModel()
            self._road = RoadSurfaceModel()

    def run(self):
        while True:
            try:
                msg = self._req.get(timeout=5.0)
            except Empty:
                continue
            msg_type = msg.get('type')
            payload = msg.get('payload')
            if msg_type == MSG_TYPE_STOP:
                self._resp.put({'type': MSG_TYPE_RESULT,
                                'payload': {'task_id': '__stop__', 'ok': True}})
                return
            if msg_type == MSG_TYPE_PING:
                self._resp.put({'type': MSG_TYPE_PONG,
                                'payload': {'ts': time.time(),
                                            'pid': os.getpid()}})
                continue
            if msg_type == MSG_TYPE_TASK:
                try:
                    self._lazy_init()
                    result = self._compute(DynamicsTask(**payload))
                    self._resp.put({
                        'type': MSG_TYPE_RESULT,
                        'payload': {
                            'task_id': payload['task_id'],
                            'ok': True,
                            'result': result
                        }
                    })
                except Exception as e:
                    self._resp.put({
                        'type': MSG_TYPE_RESULT,
                        'payload': {
                            'task_id': payload.get('task_id', 'unknown'),
                            'ok': False,
                            'error': str(e),
                            'traceback': traceback.format_exc()
                        }
                    })

    def _compute(self, task: DynamicsTask) -> Dict[str, Any]:
        # 完全复刻 MultiVehicleSteeringModel.compute_stability() 逻辑
        _, _, cfg = self._steering._build_models(task.vehicle_type)
        if not cfg:
            raise ValueError(f'Unknown vehicle_type: {task.vehicle_type}')
        speed_capped = min(task.speed_mps, cfg.max_speed_mps)
        road_effect = self._road.compute_effects(
            task.road_type, cfg.dynamics, task.friction_coeff
        )
        mod_dyn = VehicleDynamicsParams(
            wheelbase=max(0.01, cfg.dynamics.wheelbase),
            track_width=max(0.01, cfg.dynamics.track_width),
            cg_height=cfg.dynamics.cg_height,
            cg_longitudinal=cfg.dynamics.cg_longitudinal,
            cg_lateral=cfg.dynamics.cg_lateral,
            roll_center_height=cfg.dynamics.roll_center_height,
            mass=cfg.dynamics.mass,
            yaw_inertia=cfg.dynamics.yaw_inertia,
            roll_stiffness=cfg.dynamics.roll_stiffness,
            damping_ratio=cfg.dynamics.damping_ratio,
            wheel_radius=cfg.dynamics.wheel_radius,
            cornering_stiffness_front=max(500.0, road_effect.effective_cornering_stiffness_front),
            cornering_stiffness_rear=max(500.0, road_effect.effective_cornering_stiffness_rear)
        )
        cargo = CargoConfig(
            mass=task.cargo_mass,
            offset_lateral=task.cargo_offset_lateral,
            offset_longitudinal=task.cargo_offset_longitudinal,
            offset_height=task.cargo_offset_height,
            shift_stiffness=task.cargo_shift_stiffness,
            shift_damping=task.cargo_shift_damping,
            shift_dynamics=task.cargo_shift_dynamics
        )
        analyzer = StabilityAnalyzer(mod_dyn, cargo)
        max_angle = cfg.max_steering_angle_deg
        pole_clamped = max(-max_angle, min(max_angle, task.pole_angle_deg))
        stab = analyzer.analyze(
            speed=speed_capped,
            pole_angle_deg=pole_clamped,
            roll_angle_deg=task.roll_angle_deg,
            slip_rate=0.1,
            friction_coeff=road_effect.friction_coeff,
            dt=0.05,
            vertical_accel=road_effect.vibration_acceleration
        )
        ssf = mod_dyn.track_width / (2 * stab.effective_cg_height)
        return {
            'vehicle_type': task.vehicle_type,
            'vehicle_name': cfg.name,
            'roll_angle': stab.roll_angle,
            'roll_rate': stab.roll_rate,
            'yaw_rate': stab.yaw_rate,
            'lateral_acceleration': stab.lateral_acceleration,
            'roll_center_height': stab.roll_center_height,
            'rollover_risk': stab.rollover_risk,
            'stability_index': stab.stability_index,
            'understeer_gradient': stab.understeer_gradient,
            'critical_speed': stab.critical_speed,
            'effective_cg_height': stab.effective_cg_height,
            'effective_cg_lateral': stab.effective_cg_lateral,
            'effective_cg_longitudinal': stab.effective_cg_longitudinal,
            'effective_yaw_inertia': stab.effective_yaw_inertia,
            'cargo_shift_lateral': stab.cargo_shift_lateral,
            'cargo_shift_vertical': stab.cargo_shift_vertical,
            'ssf_static': ssf,
            'max_speed_mps': cfg.max_speed_mps,
            'mass': cfg.dynamics.mass,
            'cg_height': cfg.dynamics.cg_height,
            'wheelbase': cfg.dynamics.wheelbase,
            'track_width': cfg.dynamics.track_width,
            'propulsion': cfg.propulsion,
            'friction_coeff_used': road_effect.friction_coeff,
            'road_type': task.road_type,
            'vibration_level': road_effect.vibration_acceleration
        }


def _worker_entry(req_q: mp.Queue, resp_q: mp.Queue):
    """子进程入口（顶层函数，pickle友好）"""
    server = _DynamicsWorkerServer(req_q, resp_q)
    server.run()


class DynamicsWorkerClient:
    """主进程端客户端：submit/poll/stop"""

    def __init__(self, start: bool = True):
        self._ctx = MP_SPAWN_CTX
        self._req: Optional[mp.Queue] = None
        self._resp: Optional[mp.Queue] = None
        self._proc: Optional[mp.Process] = None
        self._pending: Dict[str, Any] = {}
        self._completed: Dict[str, Any] = {}
        if start:
            self.start()

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._req = self._ctx.Queue()
        self._resp = self._ctx.Queue()
        self._proc = self._ctx.Process(
            target=_worker_entry, args=(self._req, self._resp),
            name='DynamicsWorker', daemon=True
        )
        self._proc.start()
        # 启动握手
        self._req.put({'type': MSG_TYPE_PING, 'payload': None})
        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                msg = self._resp.get(timeout=0.2)
                if msg['type'] == MSG_TYPE_PONG:
                    return
            except Empty:
                continue
        raise RuntimeError('DynamicsWorker failed to start within 5s')

    def stop(self, timeout: float = 3.0) -> None:
        if not self.running:
            return
        try:
            self._req.put({'type': MSG_TYPE_STOP, 'payload': None})
        except Exception:
            pass
        self._proc.join(timeout=timeout)
        if self._proc.is_alive():
            self._proc.terminate()
        self._proc = None
        self._req = None
        self._resp = None

    def submit(self, task: DynamicsTask) -> str:
        if not self.running:
            self.start()
        self._req.put({'type': MSG_TYPE_TASK, 'payload': asdict(task)})
        self._pending[task.task_id] = task
        return task.task_id

    def poll(self, timeout: float = 0.0) -> List[Tuple[str, bool, Any]]:
        """返回 [(task_id, ok, result_or_error)]"""
        if not self.running:
            return []
        out = []
        deadline = time.time() + timeout
        while True:
            remaining = max(0.0, deadline - time.time())
            try:
                msg = self._resp.get(timeout=remaining if remaining > 0 else 0.01)
            except Empty:
                break
            if msg['type'] != MSG_TYPE_RESULT:
                continue
            p = msg['payload']
            tid = p['task_id']
            if tid in self._pending:
                del self._pending[tid]
            ok = p.get('ok', False)
            if ok:
                self._completed[tid] = p['result']
                out.append((tid, True, p['result']))
            else:
                err = p.get('error', 'Unknown error')
                self._completed[tid] = {'error': err}
                out.append((tid, False, err))
            if remaining <= 0:
                break
        return out

    def compute_sync(self, task: DynamicsTask, timeout: float = 10.0) -> Dict[str, Any]:
        """同步调用（便捷 API）"""
        tid = self.submit(task)
        deadline = time.time() + timeout
        while time.time() < deadline:
            results = self.poll(timeout=0.2)
            for task_id, ok, payload in results:
                if task_id == tid:
                    if ok:
                        return payload
                    raise RuntimeError(f'Dynamics task failed: {payload}')
        raise TimeoutError(f'Dynamics task {tid} timed out after {timeout}s')

    def get_result(self, task_id: str) -> Optional[Any]:
        return self._completed.get(task_id)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()


if __name__ == '__main__':
    # 命令行启动模式：单独运行Worker进程（常驻）
    print(f'[DynamicsWorker standalone mode] PID={os.getpid()}')
    req = MP_SPAWN_CTX.Queue()
    resp = MP_SPAWN_CTX.Queue()
    server = _DynamicsWorkerServer(req, resp)
    server.run()
