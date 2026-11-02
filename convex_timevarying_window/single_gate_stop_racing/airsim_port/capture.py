"""Read-only close-passage observer plus native rendered screenshots."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from .rpc import Client
from .scene import Scene
from .flight import physical_state,save_image


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--result-dir',required=True)
    parser.add_argument('--wall-limit',type=float,default=1200.)
    args=parser.parse_args();out=Path(args.result_dir)/'passage_views';out.mkdir(exist_ok=False)
    c=Client();scene=Scene(c);began=time.monotonic();stamp=None;captures=[];highres={}
    while time.monotonic()-began<args.wall_limit:
        result=json.loads((out.parent/'result.json').read_text())
        if result['status']!='running':break
        raw=c.call('getMultirotorState','RaceDrone');state=physical_state(raw,scene.origin)
        close=[]
        for i,w in enumerate(scene.track.windows):
            normal=w.state_at(0.)[1][:,2]
            if np.linalg.norm(state[:3]-w.center0)<8 and abs((state[:3]-w.center0)@normal)<1.5:
                close.append(i+1)
        if close and raw['timestamp']!=stamp:
            stamp=raw['timestamp'];save_image(c,out/f'{stamp}.png')
            # UE's own screenshot command captures the actual main chase view.
            # At most one full-resolution image per 0.1s native time per gate.
            # Capturing every wall-clock poll would swamp rendering/storage.
            if any((stamp-highres.get(i,0))*1e-9>.1 for i in close):
                c.call('simRunConsoleCommand','HighResShot 1')
                for i in close:highres[i]=stamp
            captures.append(dict(timestamp=stamp,near_gates=close,position=state[:3].tolist()))
        time.sleep(.08)
    (out/'captures.json').write_text(json.dumps(captures,indent=2))
    print('Passage captures',len(captures),flush=True)


if __name__=='__main__':main()
