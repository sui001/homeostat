import sys, numpy as np, plan, wrist, math
from homeostat_path import path_mm
plan.HOTEND[:] = plan.HOTEND_LONG
scale, x, yaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
P,_=path_mm(0,30,scale,2.0); W=plan.place(P,(x,0),0,0,yaw)
s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(W,axis=0),axis=1))]; S=np.arange(0,s[-1]+1e-9,0.001); dense=np.c_[[np.interp(S,s,W[:,k]) for k in range(3)]].T
D=plan.fib_dirs(900)
for i in range(0,len(W),2):
  tip=W[i]; m=S<=s[i]+1e-9; tan=W[min(i+1,len(W)-1)]-W[i-1]; tan/=np.linalg.norm(tan)
  h=(plan.clearance_all_dirs(D,tip,dense[m],s[i]-S[m],0.0)>=0)&(D@tan<=0.5)
  a=plan.arm_clearance_all_dirs(D,tip,dense[m],0.0)>=0
  j5,reach=wrist.j5_est(tip,D); j3=wrist.j3_est(tip,D); lim=reach&(j5<=plan.J5_MAX)&(j5>=plan.J5_MIN)&(j3<=plan.J3_MAX)
  if not (h&a&lim).any():
    print("yaw",yaw,"x",x,"blocked at",i,"of",len(W),"tip",np.round(tip,3),"hot-clear",h.sum(),"hot&arm",(h&a).sum(),"hot&lim",(h&lim).sum())
    for k in np.nonzero(h)[0][:6]: print("   dir",np.round(D[k],2),"J5 %.0f J3 %.0f reach %s armclear %s"%(j5[k],j3[k],reach[k],a[k]))
    break
