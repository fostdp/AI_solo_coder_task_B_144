import sys
sys.path.insert(0, 'services')

print('--- Test 1: RoadSurfaceModel ---')
from common.extended_models import RoadSurfaceModel
r = RoadSurfaceModel()
rt = r.list_road_types()
print('  count:', len(rt))
print('  first:', rt[0] if rt else None)
print('  type of first:', type(rt[0]).__name__ if rt else 'N/A')

print()
print('--- Test 2: MultiVehicleSteeringModel ---')
from common.extended_models import MultiVehicleSteeringModel
m = MultiVehicleSteeringModel()
print('  _vehicle_configs type:', type(m._vehicle_configs).__name__)
print('  _vehicle_configs keys count:', len(m._vehicle_configs))
for k in list(m._vehicle_configs.keys())[:3]:
    v = m._vehicle_configs[k]
    print('    key=%s val_type=%s' % (k, type(v).__name__))

vt = m.list_vehicle_types()
print('  list_vehicle_types count:', len(vt))
