"""Native SimpleFlight takeoff/hover/land smoke test, NOT a racing experiment."""
import json
from pathlib import Path
import time
from .rpc import Client


def main():
    out = Path(__file__).parent/'results'/'native_smoke_20261010'
    out.mkdir(parents=True, exist_ok=False)
    client = Client(timeout=30.)
    name = 'RaceDrone'
    samples = []
    result = {'test': 'native takeoff-hover-land only', 'seven_gate_race': False,
              'vehicle': 'AirSim v1.8.1 SimpleFlight GenericQuad',
              'pose_teleport_for_flight': False}
    try:
        client.call('enableApiControl', True, name)
        if not client.call('armDisarm', True, name):
            raise RuntimeError('Arming rejected')
        result['takeoff_return'] = client.call('takeoff', 20., name)
        for _ in range(25):
            samples.append(client.call('getMultirotorState', name))
            time.sleep(.2)
        result['land_return'] = client.call('land', 20., name)
        result['final_state'] = client.call('getMultirotorState', name)
        result['collision_detected_during_hover'] = any(
            s['collision']['has_collided'] for s in samples)
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        (out/'states.json').write_text(json.dumps(samples, indent=2)+'\n')
        client.close()
    print(json.dumps({k:v for k,v in result.items() if k != 'final_state'}, indent=2))


if __name__ == '__main__':
    main()
