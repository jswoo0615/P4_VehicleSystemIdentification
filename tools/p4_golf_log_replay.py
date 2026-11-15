#!/usr/bin/env python3
"""VW Golf 75--100 s steering-log replay; standalone diagnostic, not C++ parity."""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

A_K, A_TAU, A_B = 3.720892737136853, .12650895102857465, -.002017215946416358
AS_L, AS_KV, AS_TAU, AS_B = 2.63, .005, .14194537768844415, -.009035082107056886
CF, CR, IZ_M, LF, LR = 48.37692797945161, 51.01006700133779, 2.530180722165194, 1., 1.63
G = 9.80665

def simulate(csv: Path, start: float, end: float, decimation: int = 4):
    df = pd.read_csv(csv, comment='#')
    df = df[(df.t_s >= start) & (df.t_s <= end) & (df.Vx_mps >= 5)].iloc[::decimation].reset_index(drop=True)
    if len(df)<2: raise ValueError('Need at least 2 samples above 5 m/s in interval')
    t = df.t_s.to_numpy(float); v = df.Vx_mps.to_numpy(float)
    steer=df.roadWheelAngle_rad.to_numpy(float);bank=df.bank_rad.to_numpy(float)
    r_meas=df.yawRate_radps.to_numpy(float);vy_meas=df.Vy_mps.to_numpy(float)
    if np.any(np.diff(t)>.025): raise ValueError('Discontinuity after filtering: split segments before replay')
    r_a=np.empty_like(t);r_as=np.empty_like(t);b0=np.empty((len(t),2))
    r_a[0]=r_as[0]=r_meas[0];b0[0]=[vy_meas[0],r_meas[0]]
    for i in range(1,len(t)):
        dt=t[i]-t[i-1]; vm=.5*(v[i-1]+v[i]); d=.5*(steer[i-1]+steer[i]);bankm=.5*(bank[i-1]+bank[i])
        def midpoint_1d(prev,tau,drive):
            h=dt/(2*tau);return ((1-h)*prev+2*h*drive)/(1+h)
        r_a[i]=midpoint_1d(r_a[i-1],A_TAU,A_K*d+A_B)
        r_as[i]=midpoint_1d(r_as[i-1],AS_TAU,vm/(AS_L+AS_KV*vm*vm)*d+AS_B)
        mat=np.array([[-(CF+CR)/vm,(-LF*CF+LR*CR)/vm-vm], [(-LF*CF+LR*CR)/(IZ_M*vm),-(LF**2*CF+LR**2*CR)/(IZ_M*vm)]])
        forcing=np.array([CF*d+G*np.sin(bankm),LF*CF/IZ_M*d])
        b0[i]=np.linalg.solve(np.eye(2)-dt*mat/2,(np.eye(2)+dt*mat/2)@b0[i-1]+dt*forcing)
    return pd.DataFrame({'t_s':t,'steer_rad':steer,'vx_mps':v,'bank_rad':bank,'yaw_measured_radps':r_meas,'yaw_A_radps':r_a,'yaw_ASpeed_radps':r_as,'yaw_B0_radps':b0[:,1],'vy_measured_mps':vy_meas,'vy_B0_mps':b0[:,0]})

def main():
    ap=argparse.ArgumentParser();ap.add_argument('csv',type=Path);ap.add_argument('--start',type=float,default=75);ap.add_argument('--end',type=float,default=100);ap.add_argument('--decimation',type=int,default=4);ap.add_argument('--out',type=Path,default=Path('output/golf_replay_75_100'));args=ap.parse_args()
    df=simulate(args.csv,args.start,args.end,args.decimation);args.out.parent.mkdir(parents=True,exist_ok=True)
    df.to_csv(str(args.out)+'.csv',index=False)
    def rmse(k,target): return float(np.sqrt(np.mean((df[k]-df[target])**2)))
    print('Replay metrics (full-data fitted coefficients; in-sample diagnostic only)')
    for model,col in [('A','yaw_A_radps'),('A-Speed','yaw_ASpeed_radps'),('B0','yaw_B0_radps')]:print(f'{model:8s} yaw RMSE {rmse(col,"yaw_measured_radps"):.6f} rad/s')
    print(f'B0       Vy RMSE  {rmse("vy_B0_mps","vy_measured_mps"):.6f} m/s')
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
    tt=df.t_s.to_numpy()
    axes[0].plot(tt,np.rad2deg(df.steer_rad),lw=1,label='Measured road-wheel angle');axes[0].set_ylabel('Steering (deg)');axes[0].legend(loc='upper right')
    axes[1].plot(tt,df.yaw_measured_radps,'k',lw=1.6,label='Measured yaw')
    for key,label in [('yaw_A_radps','Model A'),('yaw_ASpeed_radps','A-Speed'),('yaw_B0_radps','Model B0')]:axes[1].plot(tt,df[key],lw=1,label=label,alpha=.85)
    axes[1].set_ylabel('Yaw rate (rad/s)');axes[1].legend(loc='upper right',ncol=2)
    axes[2].plot(tt,df.vy_measured_mps,'k',lw=1.6,label='Measured Vy');axes[2].plot(tt,df.vy_B0_mps,lw=1.2,label='Model B0 Vy');axes[2].set_ylabel('Lateral speed (m/s)');axes[2].set_xlabel('Time (s)');axes[2].legend(loc='upper right')
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle(f'VW Golf steering-input log replay | {args.start:g}–{args.end:g} s | {len(df)} frames')
    fig.savefig(str(args.out)+'.png',dpi=170);print('Saved',str(args.out)+'.png')
if __name__=='__main__':main()
