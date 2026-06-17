import json
import os
import re
from typing import Any, Dict
from dataclasses import dataclass


ENV_PATTERN = re.compile(r'\$\{([^}:]+)(?::-(.*?))?\}')


def _replace_env_vars(value: Any) -> Any:
    if isinstance(value, str):
        def replace_match(match):
            env_name = match.group(1)
            default = match.group(2) if match.group(2) is not None else ''
            env_value = os.environ.get(env_name, default)
            return env_value
        return ENV_PATTERN.sub(replace_match, value)
    elif isinstance(value, dict):
        return {k: _replace_env_vars(v) for k, v in value.items()}
    elif isinstance(value, list):
        return [_replace_env_vars(item) for item in value]
    else:
        return value


class ConfigLoader:
    def __init__(self, config_dir: str = None):
        if config_dir is None:
            self.config_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), '..', '..', 'config', 'json'
            )
        else:
            self.config_dir = config_dir
        self.config_dir = os.path.abspath(self.config_dir)
        self._cache: Dict[str, Any] = {}

    def load(self, name: str) -> Dict[str, Any]:
        if name in self._cache:
            return self._cache[name]
        path = os.path.join(self.config_dir, f'{name}.json')
        with open(path, 'r', encoding='utf-8') as f:
            raw = f.read()
        data = json.loads(raw)
        data = _replace_env_vars(data)
        if name == 'system_config':
            if 'redis' in data:
                if 'port' in data['redis']:
                    data['redis']['port'] = int(data['redis']['port'])
                if 'password' in data['redis'] and data['redis']['password'] == '':
                    data['redis']['password'] = None
            if 'mqtt' in data and 'port' in data['mqtt']:
                data['mqtt']['port'] = int(data['mqtt']['port'])
        self._cache[name] = data
        return data

    def chariot_geometry(self) -> Dict[str, Any]:
        return self.load('chariot_geometry')

    def vehicle_dynamics(self) -> Dict[str, Any]:
        return self.load('vehicle_dynamics')

    def system_config(self) -> Dict[str, Any]:
        return self.load('system_config')

    def alert_thresholds(self) -> Dict[str, Any]:
        return self.load('alert_thresholds')


_config_loader: ConfigLoader = None


def get_config_loader() -> ConfigLoader:
    global _config_loader
    if _config_loader is None:
        _config_loader = ConfigLoader()
    return _config_loader
