import json
import time
import threading
from typing import Any, Callable, Dict, Optional
from dataclasses import dataclass

try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False


class RedisClient:
    def __init__(self, host: str = 'localhost', port: int = 6379,
                 db: int = 0, password: Optional[str] = None):
        self.host = host
        self.port = port
        self.db = db
        self.password = password
        self.client: Optional[redis.Redis] = None
        self.pubsub_thread: Optional[threading.Thread] = None
        self.subscriptions: Dict[str, Callable] = {}
        self._running = False
        self._lock = threading.Lock()

    def connect(self, timeout: int = 5) -> bool:
        if not REDIS_AVAILABLE:
            print('警告: redis-py 未安装，Redis功能不可用')
            return False
        try:
            self.client = redis.Redis(
                host=self.host,
                port=self.port,
                db=self.db,
                password=self.password,
                decode_responses=True,
                socket_timeout=timeout,
                socket_connect_timeout=timeout
            )
            self.client.ping()
            print(f'Redis连接成功: {self.host}:{self.port}')
            return True
        except Exception as e:
            print(f'Redis连接失败: {e}')
            return False

    def publish(self, channel: str, message: Dict[str, Any]) -> None:
        if not self.client:
            return
        try:
            payload = json.dumps(message, ensure_ascii=False)
            self.client.publish(channel, payload)
        except Exception as e:
            print(f'发布消息失败 {channel}: {e}')

    def subscribe(self, channel: str, callback: Callable[[Dict[str, Any]], None]) -> None:
        with self._lock:
            self.subscriptions[channel] = callback
            if not self._running:
                self._start_pubsub()

    def unsubscribe(self, channel: str) -> None:
        with self._lock:
            if channel in self.subscriptions:
                del self.subscriptions[channel]

    def _start_pubsub(self) -> None:
        if not self.client:
            return
        self._running = True
        self.pubsub_thread = threading.Thread(target=self._pubsub_loop, daemon=True)
        self.pubsub_thread.start()

    def _pubsub_loop(self) -> None:
        while self._running:
            try:
                pubsub = self.client.pubsub()
                with self._lock:
                    channels = list(self.subscriptions.keys())
                if not channels:
                    time.sleep(0.1)
                    continue
                pubsub.subscribe(channels)
                for message in pubsub.listen():
                    if message['type'] == 'message':
                        try:
                            channel = message['channel']
                            data = json.loads(message['data'])
                            with self._lock:
                                cb = self.subscriptions.get(channel)
                            if cb:
                                cb(data)
                        except Exception as e:
                            print(f'处理消息失败: {e}')
            except Exception as e:
                print(f'PubSub循环异常: {e}, 5秒后重连...')
                time.sleep(5)

    def request_response(self, request_channel: str, response_channel: str,
                         request: Dict[str, Any], timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        if not self.client:
            return None
        try:
            request_id = f'req_{int(time.time()*1000)}_{id(request)}'
            request_with_id = {**request, 'request_id': request_id}

            pubsub = self.client.pubsub()
            pubsub.subscribe(response_channel)

            self.publish(request_channel, request_with_id)

            start = time.time()
            while time.time() - start < timeout:
                message = pubsub.get_message(timeout=0.1)
                if message and message['type'] == 'message':
                    data = json.loads(message['data'])
                    if data.get('request_id') == request_id:
                        pubsub.unsubscribe()
                        return data
            pubsub.unsubscribe()
            return None
        except Exception as e:
            print(f'请求响应失败: {e}')
            return None

    def close(self) -> None:
        self._running = False
        if self.client:
            self.client.close()
