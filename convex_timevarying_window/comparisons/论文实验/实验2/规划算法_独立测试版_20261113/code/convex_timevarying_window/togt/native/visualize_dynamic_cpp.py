#!/usr/bin/env python3
"""Render the pure-C++ H=3 rolling trajectory in the paper GIF style."""
from pathlib import Path
import argparse
import numpy as np
import imageio.v2 as imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

CENTERS=np.array([[-1.1,-1.6,3.6],[9.2,6.6,1.0],[9.2,-4,1.2],[-4.5,-6,3.5],[-4.5,-6,.8],[4.75,-.9,1.2],[-2.8,6.8,1.2]])
BASE=np.deg2rad(np.array([[0,-90,0],[0,-90,-20],[0,-90,-130],[0,-90,180],[0,-90,0],[0,-90,70],[0,-90,200]],float))
SHAPES=[{"type":"RectanglePrisma","width":2.4,"height":2.4} for _ in CENTERS]
SPEED_SCALE=1.0

def rot(a):
    x,y,z=a; cx,sx=np.cos(x),np.sin(x); cy,sy=np.cos(y),np.sin(y); cz,sz=np.cos(z),np.sin(z)
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])
def motion(i,t):
    at=np.array([0.,.45+.05*((3*i)%6),0.])
    ar=np.deg2rad([15+4*(i%6),0.,0.])
    phase=-1.3+.37*i; phases=np.array([phase,phase+.73,phase+1.41])
    base=rot(BASE[i]); tangent=base[:,0]; normal=base[:,2]
    pivot_local=local_boundary(SHAPES[i],safe=True)[:-1].mean(0)
    center=CENTERS[i]+base@pivot_local+tangent*at[1]*np.sin(phase+2*np.pi*t*SPEED_SCALE/(9.5+.47*(i%11)))
    angle=ar[0]*np.sin(phase+.73+2*np.pi*t*SPEED_SCALE/(8+.43*((3*i)%13)))
    K=np.array([[0,-normal[2],normal[1]],[normal[2],0,-normal[0]],[-normal[1],normal[0],0]])
    dynamic=np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K)
    return center,dynamic@base,pivot_local
def local_boundary(shape,safe=False):
    kind=shape["type"]
    if kind=="TrianglePrisma":
        margin=shape.get("margin",0.) if safe else 0.;hw=.5*(shape["width"]-margin);hh=.5*(shape["height"]-margin)
        q=np.array([[-hh,hw,0],[hh,0,0],[-hh,-hw,0]])
    elif kind=="RectanglePrisma":
        mw=shape.get("marginW",0.) if safe else 0.;mh=shape.get("marginH",0.) if safe else 0.;hw=.5*(shape["width"]-mw);hh=.5*(shape["height"]-mh)
        q=np.array([[-hh,hw,0],[-hh,-hw,0],[hh,-hw,0],[hh,hw,0]])
    elif kind=="PentagonPrisma":
        radius=shape["radius"]-(shape.get("margin",0.) if safe else 0.);angles=np.deg2rad([144,72,0,-72,-144])
        q=np.c_[radius*np.cos(angles),radius*np.sin(angles),np.zeros(5)]
    elif kind=="HexagonPrisma":
        side=shape["side"]-(shape.get("margin",0.) if safe else 0.);height=.5*side*np.tan(np.pi/3)
        q=np.array([[-height,.5*side,0],[0,side,0],[height,.5*side,0],[height,-.5*side,0],[0,-side,0],[-height,-.5*side,0]])
    else: raise ValueError(f"unsupported visualization shape: {kind}")
    return np.vstack([q,q[0]])
def boundary(i,t):
    q=local_boundary(SHAPES[i])
    c,R,pivot_local=motion(i,t);return c+(q-pivot_local)@R.T
def drone(ax,p,v,a,size=.32):
    z=a+np.array([0,0,9.8066]); z/=max(np.linalg.norm(z),1e-9)
    x=np.array([v[0],v[1],0.]); x=x/max(np.linalg.norm(x),1e-9) if np.linalg.norm(x)>1e-9 else np.array([1.,0,0])
    y=np.cross(z,x); y/=max(np.linalg.norm(y),1e-9); x=np.cross(y,z)
    for u in (x+y,x-y):
        u=u/np.linalg.norm(u)*size
        ax.plot([p[0]-u[0],p[0]+u[0]],[p[1]-u[1],p[1]+u[1]],[p[2]-u[2],p[2]+u[2]],color="#343a40",lw=2)
    ax.scatter(*p,s=18,color="#f28e2b",edgecolor="#343a40",zorder=8)
def main():
    global CENTERS,BASE,SHAPES,SPEED_SCALE
    p=argparse.ArgumentParser();p.add_argument("csv",type=Path);p.add_argument("gif",type=Path);p.add_argument("--track",type=Path);p.add_argument("--speed-scale",type=float,default=1.0);a=p.parse_args();SPEED_SCALE=a.speed_scale
    if a.track:
        scene=yaml.safe_load(a.track.read_text()); names=scene["orders"]
        CENTERS=np.asarray([scene[n]["position"] for n in names],float)
        BASE=np.deg2rad(np.asarray([scene[n]["rpy"] for n in names],float))
        SHAPES=[scene[n] for n in names]
        start=np.asarray(scene["initState"]["pos"],float)
    else: start=np.array([-5.,4.5,1.2])
    d=np.genfromtxt(a.csv,delimiter=",",names=True); times=d["time"]; pos=np.c_[d["px"],d["py"],d["pz"]]; vel=np.c_[d["vx"],d["vy"],d["vz"]]; acc=np.c_[d["ax"],d["ay"],d["az"]]
    frame_times=np.linspace(0,times[-1],72); frames=[]
    for t in frame_times:
        fig=plt.figure(figsize=(6.4,5.2),constrained_layout=True); ax=fig.add_subplot(111,projection="3d"); k=min(np.searchsorted(times,t),len(times)-1)
        ax.plot(*pos.T,color="0.80",lw=1); ax.plot(*pos[:k+1].T,color="#0072bd",lw=2.2); drone(ax,pos[k],vel[k],acc[k])
        ax.scatter(*start,facecolor="white",edgecolor="#2a9d8f",lw=1.5,s=38);ax.text(*start,"  S/F",color="#1f6f65",fontsize=7,fontweight="bold")
        for i in range(len(CENTERS)):
            b=boundary(i,t);ax.plot(*b.T,color="#343a40",lw=3.4,solid_capstyle="round",alpha=.88);ax.plot(*b.T,color="#f28e2b",lw=1.8,solid_capstyle="round",alpha=.94);c=b[:-1].mean(0)
            if len(CENTERS)<=10 or i%5==0: ax.text(*c,str(i+1),color="white",fontsize=6.2,fontweight="bold",ha="center",va="center",bbox=dict(boxstyle="circle,pad=.18",fc="#343a40",ec="#f28e2b",lw=.7,alpha=.92))
        low=np.minimum(pos.min(0),CENTERS.min(0))-2; high=np.maximum(pos.max(0),CENTERS.max(0))+2
        xx,yy=np.meshgrid([low[0],high[0]],[low[1],high[1]]);ax.plot_surface(xx,yy,np.zeros_like(xx),color="#dfe5e8",alpha=.18,shade=False)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2],max(6.,high[2])),xlabel="x [m]",ylabel="y [m]",zlabel="z [m]",title=f"{len(CENTERS)} dynamic windows · Rolling-PVAJ (H=3)  |  t = {t:.2f} s")
        ax.set_facecolor("#f7f9fb");ax.grid(True,alpha=.28,lw=.5);ax.view_init(elev=24,azim=-61);fig.canvas.draw();frames.append(np.asarray(fig.canvas.buffer_rgba())[...,:3].copy());plt.close(fig)
    a.gif.parent.mkdir(parents=True,exist_ok=True);imageio.mimsave(a.gif,frames,duration=1/12,loop=0)
if __name__=="__main__":main()
