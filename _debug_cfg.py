import sys
sys.path.insert(0, 'services')
from common.config_loader import get_config_loader
loader = get_config_loader()
data = loader.load('vehicle_types')
print('type:', type(data))
print('keys:', list(data.keys())[:3])
for k in list(data.keys())[:2]:
    v = data[k]
    print('  %s: type=%s' % (k, type(v).__name__))
    if isinstance(v, dict):
        print('    name:', v.get('name'))
    else:
        print('    value preview:', str(v)[:80])
