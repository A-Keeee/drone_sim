#!/usr/bin/env python3
import argparse, csv, math
import matplotlib.pyplot as plt
def main():
    p=argparse.ArgumentParser();p.add_argument('csv');p.add_argument('--prefix',default='experiment');p.add_argument('--goal',nargs=3,type=float,default=[0,0,1.5]);a=p.parse_args()
    with open(a.csv) as f:r=list(csv.DictReader(f));t=[float(x['t']) for x in r]
    xyz=[[float(x[k]) for x in r] for k in ('x','y','z')]
    fig,ax=plt.subplots();[ax.plot(t,[float(x[k]) for x in r],label=k) for k in ('x','y','z')];ax.legend();ax.set(xlabel='time [s]',ylabel='position [m]');fig.savefig(a.prefix+'_position.png',dpi=160)
    fig,ax=plt.subplots();[ax.plot(t,[float(x[k]) for x in r],label=k) for k in ('rpm1','rpm2','rpm3','rpm4')];ax.legend();ax.set(xlabel='time [s]',ylabel='RPM');fig.savefig(a.prefix+'_rpm.png',dpi=160)
    error=[math.dist(p,a.goal) for p in zip(*xyz)]
    fig,ax=plt.subplots();ax.plot(t,error);ax.axhline(.3,color='r',linestyle='--');ax.set(xlabel='time [s]',ylabel='position error [m]');fig.savefig(a.prefix+'_error.png',dpi=160)
    fig,axs=plt.subplots(1,2,figsize=(10,4));axs[0].plot(xyz[0],xyz[1]);axs[0].set(xlabel='x [m]',ylabel='y [m]',aspect='equal');axs[1].plot(xyz[0],xyz[2]);axs[1].set(xlabel='x [m]',ylabel='z [m]');fig.tight_layout();fig.savefig(a.prefix+'_trajectory.png',dpi=160)
    boxes=[(1.5,0,1.2,.6,2,2.4),(3,-1.2,2,.8,1.6,4),(4,1.2,2,.8,1.6,4),(5.4,0,3.7,.8,3.5,1),(6.8,0,1.3,.8,4.5,2.6)]
    def box_distance(point,box): return math.sqrt(sum(max(abs(point[i]-box[i])-box[i+3]/2,0)**2 for i in range(3)))
    clearance=[min(box_distance(point,b) for b in boxes) for point in zip(*xyz)]
    fig,ax=plt.subplots();ax.plot(t,clearance);ax.axhline(.5,color='r',linestyle='--');ax.set(xlabel='time [s]',ylabel='obstacle clearance [m]');fig.savefig(a.prefix+'_clearance.png',dpi=160)
if __name__=='__main__':main()
