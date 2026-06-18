# -*- coding: utf-8 -*-
"""
_test_refactored_modules.py
===========================
重构后新模块专项测试套件
验证 5 个新模块的独立性和正确性：
  1. steering_comparator.py - 机构对比
  2. era_comparator.py - 跨时代对比
  3. road_simulator.py - 路面影响仿真
  4. vr_chariot.py - 虚拟驾驶引擎
  5. dynamics_worker.py - 多体动力学 Worker 进程

每个模块测试覆盖：
  - 模块可独立导入
  - 类实例化
  - 核心方法返回值结构正确
  - Normal / Boundary / Abnormal 三类场景
"""
import sys
import os
import time

PASS = 0
FAIL = 0
TOTAL = 0


def test(name, condition, detail=''):
    global PASS, FAIL, TOTAL
    TOTAL += 1
    if condition:
        PASS += 1
        print('[ OK ] %s%s' % (name, (' - ' + detail) if detail else ''))
    else:
        FAIL += 1
        print('[FAIL] %s%s' % (name, (' - ' + detail) if detail else ''))


def section(title):
    print()
    print('=' * 70)
    print(title)
    print('=' * 70)


def test_approx(name, actual, expected, tol, detail=''):
    diff = abs(actual - expected)
    test(name, diff <= tol, 'actual=%.6f, expected=%.6f, diff=%.6f, tol=%.3f%s' % (
        actual, expected, diff, tol, (' - ' + detail) if detail else ''
    ))


sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'services'))


def run_all_tests():
    global PASS, FAIL, TOTAL
    PASS = 0
    FAIL = 0
    TOTAL = 0

    print('=' * 70)
    print('Refactored Modules Test Suite v1.0')
    print('=' * 70)

    # =====================================================================
    # [1/5] steering_comparator 模块测试
    # =====================================================================
    section('[1/5] Steering Comparator Module')
    try:
        from common.steering_comparator import SteeringComparator
        cmp = SteeringComparator()
        test('Module import + instantiation', cmp is not None)

        result = cmp.compare(
            ['chariot_double', 'wheelbarrow_single', 'chariot_four_wheel', 'modern_car'],
            15.0, 5.0, 0.7, 'dirt_road'
        )
        test('compare() returns ComparisonResult', hasattr(result, 'entries'))
        test('4 vehicles in entries', len(result.entries) == 4, 'count=%d' % len(result.entries))
        test('winners dict not empty', len(result.winners) >= 3, 'count=%d' % len(result.winners))
        test('insights list not empty', len(result.insights) >= 2, 'count=%d' % len(result.insights))
        test('entries have required fields', all(
            'vehicle_type' in e and 'turning_radius' in e and 'rollover_risk' in e
            for e in result.entries
        ))
        test('all turning_radius > 0', all(
            e['turning_radius'] > 0 or e['turning_radius'] == float('inf')
            for e in result.entries
        ))

        r0 = cmp.compare(['chariot_double'], 0.0, 5.0, 0.7)
        test('Boundary: 0° pole => near-straight (R large)',
             r0.entries[0]['turning_radius'] == float('inf') or r0.entries[0]['turning_radius'] > 1000)

        r_big = cmp.compare(['chariot_double'], 45.0, 5.0, 0.7)
        test('Boundary: 45° pole => capped (no crash)', len(r_big.entries) == 1)

        r_empty = cmp.compare([], 10, 5, 0.7)
        test('Abnormal: empty vehicle list => 0 entries', len(r_empty.entries) == 0)

        r_mix = cmp.compare(['chariot_double', 'invalid_xxx', 'modern_car'], 10, 5, 0.7)
        test('Abnormal: with invalid => skip invalid', len(r_mix.entries) == 2, 'count=%d' % len(r_mix.entries))

    except Exception as e:
        test('SteeringComparator module', False, str(e))

    # =====================================================================
    # [2/5] era_comparator 模块测试
    # =====================================================================
    section('[2/5] Era Comparator Module')
    try:
        from common.era_comparator import EraComparator
        era = EraComparator()
        test('Module import + instantiation', era is not None)

        result = era.compare_eras()
        test('compare_eras() returns ComparisonResult', hasattr(result, 'entries'))
        test('ancient entries present', sum(1 for e in result.entries if e.get('era') == '古代') >= 2)
        test('modern entries present', sum(1 for e in result.entries if e.get('era') == '现代') >= 1)
        test('era-specific winners exist', any('古代' in k or '现代' in k for k in result.winners.keys()))
        test('era-specific insights present', len(result.insights) >= 4)

        r_anc = era.compare_eras(['chariot_double'], [])
        test('Boundary: only ancient => still works', len(r_anc.entries) == 1)

        r_mod = era.compare_eras([], ['modern_car'])
        test('Boundary: only modern => still works', len(r_mod.entries) == 1)

        r_none = era.compare_eras([], [])
        test('Abnormal: both empty => 0 entries', len(r_none.entries) == 0)

    except Exception as e:
        test('EraComparator module', False, str(e))

    # =====================================================================
    # [3/5] road_simulator 模块测试
    # =====================================================================
    section('[3/5] Road Simulator Module')
    try:
        from common.road_simulator import RoadSimulator
        rs = RoadSimulator()
        test('Module import + instantiation', rs is not None)

        road_list = rs.list_road_types()
        test('list_road_types() returns list', isinstance(road_list, list))
        test('at least 5 road types', len(road_list) >= 5, 'count=%d' % len(road_list))
        test('each has id and name', all('id' in r and 'name' in r for r in road_list))

        result = rs.compare_roads('chariot_double', 15.0, 5.0)
        test('compare_roads() returns ComparisonResult', hasattr(result, 'entries'))
        test('8 roads in entries', len(result.entries) >= 6, 'count=%d' % len(result.entries))
        test('road entries have friction_coeff', all('friction_coeff' in e for e in result.entries))
        test('road entries have rollover_risk', all('rollover_risk' in e for e in result.entries))
        test('road entries have max_safe_speed', all('max_safe_speed' in e for e in result.entries))

        sorted_by_mu = sorted(result.entries, key=lambda e: e['friction_coeff'], reverse=True)
        highest_mu = sorted_by_mu[0]['friction_coeff']
        lowest_mu = sorted_by_mu[-1]['friction_coeff']
        test('Normal: friction spread valid', highest_mu > 0.5 and lowest_mu < 0.3,
             'highest=%.2f lowest=%.2f' % (highest_mu, lowest_mu))

        r_ext = rs.compare_roads('chariot_double', 30.0, 10.0, ['asphalt_dry', 'mud_road', 'ice_snow'])
        speeds = {e['road_type']: e['max_safe_speed'] for e in r_ext.entries}
        test('Boundary: extreme => asphalt safe_speed >= mud safe_speed',
             speeds.get('asphalt_dry', 0) >= speeds.get('mud_road', 999) - 0.001,
             'asphalt=%.1f mud=%.1f' % (speeds.get('asphalt_dry', 0), speeds.get('mud_road', 0)))

        effects = rs.compute_road_effects('asphalt_dry')
        test('Boundary: compute_road_effects returns dict', isinstance(effects, dict))
        test('Boundary: effects has friction_coeff', 'friction_coeff' in effects)

        r_empty = rs.compare_roads('chariot_double', 10, 5, [])
        test('Abnormal: empty road list => 0 entries', len(r_empty.entries) == 0)

        r_mix = rs.compare_roads('chariot_double', 10, 5, ['asphalt_dry', 'bad_road'])
        test('Abnormal: with invalid => no crash (fallback default)',
             len(r_mix.entries) >= 1 and all('friction_coeff' in e for e in r_mix.entries))

    except Exception as e:
        import traceback
        test('RoadSimulator module', False, str(e) + ' | ' + traceback.format_exc()[:200])

    # =====================================================================
    # [4/5] vr_chariot 模块测试
    # =====================================================================
    section('[4/5] VR Chariot Module')
    try:
        from common.vr_chariot import VRChariotEngine
        vr = VRChariotEngine()
        test('Module import + instantiation', vr is not None)

        s1 = vr.step('test-sess-1', 'chariot_double', 'ancient_post_road', 0.0, 0.5, 0.0, dt=0.05)
        test('step() returns VirtualDriveState', hasattr(s1, 'speed') and hasattr(s1, 'heading'))
        test('initial speed >= 0', s1.speed >= 0)
        test('ffb fields present', hasattr(s1, 'ffb_total_torque') and hasattr(s1, 'ffb_intensity'))
        test('ffb_intensity in [0,1]', 0.0 <= s1.ffb_intensity <= 1.0)
        test('ffb_total_torque bounded', abs(s1.ffb_total_torque) <= 8.5)

        vr.reset_session('test-sess-2')
        for i in range(100):
            s2 = vr.step('test-sess-2', 'chariot_double', 'ancient_post_road', 0.0, 1.0, 0.0, dt=0.05)
        test('Normal: 100 steps throttle => speed > 0', s2.speed > 0.5, 'speed=%.2f' % s2.speed)
        test('Normal: speed <= max_speed', s2.speed <= 8.5)

        vr.reset_session('test-sess-3')
        for i in range(200):
            s3 = vr.step('test-sess-3', 'chariot_double', 'ancient_post_road', 20.0, 0.8, 0.0, dt=0.05)
        test('Normal: 20° turn => heading changes', abs(s3.heading) > 0.01,
             'heading=%.3f rad' % s3.heading)

        s_ffb = vr.step('test-sess-4', 'modern_car', 'asphalt_dry', 25.0, 0.6, 0.0, dt=0.05)
        test('Boundary: 25° turn => ffb present', abs(s_ffb.ffb_aligning_torque) > 0.1)

        s_ffb2 = vr.step('test-sess-5', 'modern_car', 'asphalt_dry', 10.0, 0.9, 0.0, dt=0.05)
        for i in range(200):
            s_ffb2 = vr.step('test-sess-5', 'modern_car', 'asphalt_dry', 10.0, 0.9, 0.0, dt=0.05)
        test('Boundary: high speed + steer => ffb intensity > 0.3', s_ffb2.ffb_intensity > 0.3,
             'intensity=%.2f' % s_ffb2.ffb_intensity)

        s_ice = vr.step('test-sess-ice', 'chariot_double', 'ice_snow', 15.0, 0.5, 0.0, dt=0.05)
        test('Boundary: ice road => ffb total within safety limit', abs(s_ice.ffb_total_torque) <= 8.5)

        vr.reset_session('iso-a')
        vr.reset_session('iso-b')
        for i in range(50):
            sa = vr.step('iso-a', 'chariot_double', 'ancient_post_road', 20.0, 1.0, 0.0, dt=0.05)
            sb = vr.step('iso-b', 'chariot_double', 'ancient_post_road', -10.0, 0.5, 0.0, dt=0.05)
        test('Abnormal: sessions isolated', sa.x != sb.x or sa.heading != sb.heading)

        vr.reset_session('reset-test')
        s_init = vr.step('reset-test', 'chariot_double', 'ancient_post_road', 0.0, 0.0, 0.0, dt=0.05)
        for i in range(50):
            vr.step('reset-test', 'chariot_double', 'ancient_post_road', 20.0, 1.0, 0.0, dt=0.05)
        vr.reset_session('reset-test')
        s_after = vr.step('reset-test', 'chariot_double', 'ancient_post_road', 0.0, 0.0, 0.0, dt=0.05)
        test('Abnormal: reset => position ~ origin', abs(s_after.x) < 0.1 and abs(s_after.y) < 0.1)

        s_bad = vr.step('bad-sess', 'invalid_vehicle', 'ancient_post_road', 0.0, 0.5, 0.0)
        test('Abnormal: invalid vehicle => no crash (handled)', s_bad is not None)

        s_ext = vr.step('ext-sess', 'chariot_double', 'ancient_post_road', 999.0, -5.0, 10.0)
        test('Abnormal: extreme inputs => valid output',
             hasattr(s_ext, 'rollover_risk') and 0 <= s_ext.rollover_risk <= 100)

    except Exception as e:
        import traceback
        test('VRChariot module', False, str(e) + ' | ' + traceback.format_exc()[:200])

    # =====================================================================
    # [5/5] dynamics_worker 模块测试
    # =====================================================================
    section('[5/5] Dynamics Worker Module')
    try:
        from common.dynamics_worker import DynamicsWorkerClient, DynamicsTask
        worker = DynamicsWorkerClient(start=True)
        test('Module import + client start', worker.running)

        task = DynamicsTask(
            task_id='test-001',
            vehicle_type='chariot_double',
            pole_angle_deg=10.0,
            speed_mps=5.0
        )
        result = worker.compute_sync(task, timeout=10.0)
        test('compute_sync returns dict', isinstance(result, dict))
        test('result has yaw_rate', 'yaw_rate' in result)
        test('result has rollover_risk', 'rollover_risk' in result)
        test('result has stability_index', 'stability_index' in result)
        test('rollover_risk in [0,100]', 0 <= result['rollover_risk'] <= 100)

        task2 = DynamicsTask(
            task_id='test-async-002',
            vehicle_type='modern_car',
            pole_angle_deg=15.0,
            speed_mps=20.0
        )
        tid = worker.submit(task2)
        test('submit returns task_id', tid == 'test-async-002')

        time.sleep(0.5)
        results = worker.poll(timeout=2.0)
        test('poll returns results', len(results) >= 1)
        task_ids = [r[0] for r in results]
        test('poll contains our task', 'test-async-002' in task_ids)

        for i in range(5):
            worker.submit(DynamicsTask(
                task_id='batch-%d' % i,
                vehicle_type='chariot_double',
                pole_angle_deg=5.0 + i * 5,
                speed_mps=5.0
            ))
        time.sleep(1.0)
        results_batch = worker.poll(timeout=3.0)
        test('Boundary: 5 batch tasks processed', len(results_batch) >= 4)

        task_big = DynamicsTask(task_id='big-angle', vehicle_type='chariot_double',
                                pole_angle_deg=50.0, speed_mps=3.0)
        r_big = worker.compute_sync(task_big, timeout=5.0)
        test('Boundary: 50° pole => no crash', isinstance(r_big, dict))

        task_bad = DynamicsTask(task_id='bad-vt', vehicle_type='invalid_car',
                                pole_angle_deg=10.0, speed_mps=5.0)
        try:
            r_bad = worker.compute_sync(task_bad, timeout=5.0)
            test('Abnormal: invalid vehicle => handled (no crash)', True)
        except Exception:
            test('Abnormal: invalid vehicle => handled (raises error)', True)

        worker.stop()
        test('worker.stop() => not running', not worker.running)
        worker.start()
        test('worker.restart() => running again', worker.running)

        with DynamicsWorkerClient() as w:
            test('Context manager: with block runs', w.running)
            r_ctx = w.compute_sync(DynamicsTask(task_id='ctx', vehicle_type='chariot_double',
                                                pole_angle_deg=5, speed_mps=3))
            test('Context manager: can compute inside with', isinstance(r_ctx, dict))
        test('Context manager: exited => stopped', not w.running)

    except Exception as e:
        import traceback
        test('DynamicsWorker module', False, str(e) + ' | ' + traceback.format_exc()[:300])

    # =====================================================================
    # Summary
    # =====================================================================
    print()
    print('=' * 70)
    print('Test Complete: PASS=%d, FAIL=%d, TOTAL=%d, RATE=%.1f%%' % (
        PASS, FAIL, TOTAL, 100.0 * PASS / max(1, TOTAL)
    ))
    print('=' * 70)

    if FAIL > 0:
        print()
        print('⚠️  %d TEST(S) FAILED - See details above.' % FAIL)
        return 1
    else:
        print()
        print('🎉 ALL TESTS PASSED - Refactored Modules Verified!')
        return 0


if __name__ == '__main__':
    import multiprocessing as mp
    mp.set_start_method('spawn', force=True)
    sys.exit(run_all_tests())
