import argparse
import csv
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from branchguard.simulator import run, Course, SCENARIOS, CONDITIONS
from branchguard.native import DISTANCES, SCAN, METHODS

ROOT = Path(__file__).resolve().parent
COLORS = ['#21c7d9','#f2b84b','#d481f5','#7ccc76','#ef8896','#a4bdfc','#dddddd','#a0a0a0']


def font(size):
    for name in ('C:/Windows/Fonts/segoeui.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name,size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def camera_overlay(frame,out,obs):
    image=Image.fromarray(frame).convert('RGB').resize((564,360))
    d=ImageDraw.Draw(image)
    for i in range(out.count):
        pts=[((94-80*out.paths[i].y[r]/DISTANCES[r])*3,SCAN[r]*3)
             for r in range(12) if out.paths[i].mask & (1<<r)]
        if len(pts)>1:
            d.line(pts,fill=COLORS[i],width=2)
    selected=[((94-80*out.selected.y[r]/DISTANCES[r])*3,SCAN[r]*3)
              for r in range(12) if out.selected.mask & (1<<r)]
    if len(selected)>1:
        d.line(selected,fill='#eb416b',width=5)
    for r in range(12):
        for j in range(obs.rows[r].count):
            x=(94-80*obs.rows[r].points[j].y/DISTANCES[r])*3
            y=SCAN[r]*3
            d.ellipse((x-3,y-3,x+3,y+3),fill='#ffffff',outline='#101820')
    return image


def create(scenario,condition,seed,method,prefix,gif_only=False):
    result,records,frames=run(scenario,condition,seed,method,capture=True)
    course=Course(scenario,seed)
    output=ROOT/'assets'
    output.mkdir(exist_ok=True)
    route={1:'LEFT',-1:'RIGHT',0:'STRAIGHT'}[course.route]
    panels=[]
    trajectory=[]
    for frame,out,obs,rec in frames:
        canvas=Image.new('RGB',(980,540),'#101923')
        d=ImageDraw.Draw(canvas)
        d.text((28,18),'BranchGuard',font=font(30),fill='#f5f7f9')
        d.text((28,60),f'{scenario} / {condition} / seed {seed} / {method}',font=font(17),fill='#9daebe')
        canvas.paste(camera_overlay(frame,out,obs),(28,105))
        d.text((28,479),'Gray frame -> row candidates -> paths -> selected line',font=font(15),fill='#d0dae2')
        d.text((28,509),'Candidate colours: cyan/gold/...   Selected: red',font=font(14),fill='#9daebe')
        d.text((622,103),'WORLD VIEW (evaluation only)',font=font(17),fill='#9daebe')
        def screen(x,y): return (640+(x-.2)*65,350-y*100)
        xx=np.linspace(.2,4.8,180)
        for line in course.lines(xx):
            d.line([screen(x,y) for x,y in zip(xx,line)],fill='#617487',width=2)
        d.line([screen(x,y) for x,y in zip(xx,course.target(xx))],fill='#56bfa5',width=3)
        trajectory.append(screen(rec['x'],rec['y']))
        if len(trajectory)>1: d.line(trajectory,fill='#f0b454',width=3)
        px,py=trajectory[-1]
        d.ellipse((px-5,py-5,px+5,py+5),fill='#f0b454')
        d.text((622,413),f'Instruction: {route}    t={rec["t"]:.2f}s',font=font(16),fill='#f5f7f9')
        d.text((622,441),f'{rec["decision"].upper()}   paths={out.count}',font=font(16),fill='#f0b454')
        d.text((622,470),f'Error: {rec["error"]:+.3f}m',font=font(16),fill='#9daebe')
        d.text((622,498),'Green: requested path / Gold: car',font=font(14),fill='#9daebe')
        panels.append(canvas)
    panels[-1]=panels[-1].copy()
    ImageDraw.Draw(panels[-1]).text((622,380),'Result: '+result['status'],font=font(17),fill='#f5f7f9')
    if not gif_only:
        panels[0].save(output/(prefix+'.png'))
    # A small palette keeps the demonstration practical for repository previews.
    # Numeric logs and the frames passed to C remain at full precision.
    panels=[panel.resize((735,405)).quantize(colors=32,dither=Image.Dither.NONE) for panel in panels]
    panels[0].save(output/(prefix+'.gif'),save_all=True,append_images=panels[1:]+[panels[-1]]*10,
                   duration=120,loop=0,optimize=True)
    if not gif_only:
        with (output/(prefix+'.csv')).open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=records[0]);w.writeheader();w.writerows(records)
    print(result)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--scenario',choices=SCENARIOS,default='fork_left')
    p.add_argument('--condition',choices=CONDITIONS,default='mixed')
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--method',choices=METHODS,default='branchguard')
    p.add_argument('--prefix',default='demo')
    p.add_argument('--gif-only',action='store_true')
    a=p.parse_args()
    create(a.scenario,a.condition,a.seed,a.method,a.prefix,a.gif_only)
