import sys, traceback
try:
    sys.path.insert(0, 'services')
    import json
    with open('config/json/vehicle_types.json', 'r', encoding='utf-8') as f:
        raw = f.read()
    data = json.loads(raw)
    print('STEP1 json parsed OK, type:', type(data))
    sys.stdout.flush()
    print('STEP2 keys count:', len(data))
    sys.stdout.flush()
    from common.config_loader import _replace_env_vars
    data2 = _replace_env_vars(data)
    print('STEP3 after replace, type:', type(data2))
    sys.stdout.flush()
    print('STEP4 keys:', list(data2.keys())[:5])
    sys.stdout.flush()
    for k in list(data2.keys())[:3]:
        v = data2[k]
        print('  %s: type=%s' % (k, type(v).__name__))
        sys.stdout.flush()
except Exception:
    traceback.print_exc()
    sys.exit(1)
print('ALL OK')
