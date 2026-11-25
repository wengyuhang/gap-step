"""High-contrast native gate textures; changes appearance only."""
import argparse
import json
from pathlib import Path
from PIL import Image,ImageDraw
from .rpc import Client


def color_gates(client,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    colors=[(235,45,30),(30,110,240),(245,130,20),(180,40,220),
            (10,180,150),(225,35,125),(50,70,200)]
    statuses=[]
    for i,color in enumerate(colors):
        target=(out/f'gate_{i+1}.png').resolve()
        image=Image.new('RGB',(64,64),color);draw=ImageDraw.Draw(image)
        for y in (0,32):draw.rectangle((0,y,63,y+5),fill=(250,250,250))
        image.save(target)
        for name in client.call('simListSceneObjects',f'RaceGate_{i+1:02d}_.*'):
            statuses.append(dict(name=name,success=client.call('simSetObjectMaterialFromTexture',name,str(target),0)))
    return statuses


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--outdir',required=True);a=p.parse_args()
    c=Client();statuses=color_gates(c,a.outdir)
    (Path(a.outdir)/'materials.json').write_text(json.dumps(statuses,indent=2))
    print('Native colored parts',sum(v['success'] for v in statuses),'/',len(statuses))
