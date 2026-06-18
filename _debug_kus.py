import sys
sys.path.insert(0, 'services')
from common.extended_models import MultiVehicleSteeringModel

sm = MultiVehicleSteeringModel()
for vt in ['chariot_double', 'wheelbarrow_single', 'chariot_four_wheel', 'modern_car']:
    stab = sm.compute_stability(vt, 10.0, 8.0, 0.0, 0.7, None, 'dirt_road')
    if stab:
        cfg = sm.get_vehicle_config(vt)
        print('%20s: K_us=%.3f deg/g  L=%.2fm  Cf=%.0f  Cr=%.0f  m=%.0fkg' % (
            vt,
            stab['understeer_gradient'],
            cfg.dynamics.wheelbase,
            cfg.dynamics.cornering_stiffness_front,
            cfg.dynamics.cornering_stiffness_rear,
            cfg.dynamics.mass
        ))
    else:
        print('%20s: FAILED' % vt)
