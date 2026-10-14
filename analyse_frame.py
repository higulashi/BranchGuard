"""Inspect an already calibrated 188 x 120 grayscale guide-line image."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from branchguard.native import Controller, STATUS
from make_demo import camera_overlay


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('image')
    p.add_argument('--route',choices=('left','straight','right'),default='straight')
    p.add_argument('--out',default='build/frame')
    a=p.parse_args()
    frame=np.asarray(Image.open(a.image).convert('L')).copy()
    ctl=Controller()
    route={'left':1,'straight':0,'right':-1}[a.route]
    out,obs=ctl.step(frame,route)
    dst=Path(a.out);dst.parent.mkdir(parents=True,exist_ok=True)
    camera_overlay(frame,out,obs).save(dst.with_suffix('.png'))
    decision={'status':STATUS[out.status],'paths':out.count,
              'steering_rad':out.steering,'speed_advice_m_s':out.speed,
              'note':'A single frame cannot test temporal confirmation. Calibration is synthetic.'}
    dst.with_suffix('.json').write_text(json.dumps(decision,indent=2),encoding='utf-8')
    print(json.dumps(decision,indent=2))
