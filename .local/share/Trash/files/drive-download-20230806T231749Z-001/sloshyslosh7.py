# -*- coding: utf-8 -*-
"""
Created on Sun Jul  2 09:26:28 2023

@author: mfogel
"""



import math
import numpy as np 
import matplotlib.pyplot as plt
import time
from init_imu import init_imu
from imudata import imudata
from init_maxon import init_maxon
from plotsloshy import plotsloshy
from motion_in_velocity import MotionInVelocity
from close_maxon import close_maxon


# theta = platform angle
# vel = platform angular velocity
# acc = platform angular acceleration
# u = RPM calculated for motor
# umotor = RPM commanded to motor
# Omega = RPM measured from motor


maxiter=30 # Max number of allowed iterations to meet desired angle 

umax=3500 # Max RPM of the maxon motor. SAFETY stop. 3000 is allowable. 10/7/2022 - was 4000, changed to 3000
acceleration0=4000

    

# Reaction wheel MoI
Jx=2.491E-4 # kg m^2
Jy=2.250E-4 # kg m^2
Jz=4.703E-4 # kg m^2
J=math.sqrt(Jx**2 + Jy**2 + Jz**2)
#I = 0.02344 # Full
#I = 234E-4  # Full
I = 0.01180  #Empty
#I = 118.0E-4 # Empty

# Initialization
theta_d=45 #Desired angle
tol=2  # Tolerance to angle reaching desired angle

kp0=5.5; # 50%, 75% full
kd0=2.5; # 50%, 75% full
ki0=1.0; # 50%, 75% full  - try 10^-2 or 10^-3

# Freqency Response
wn=math.sqrt(kp0/I)
eta=kd0/(2*I*wn)
p = [1, 2*eta*wn, wn**2]
r = np.roots(p)

#print('{0:2d} {1:3d} {2:4d}'.format(x, x*x, x*x*x))

print('wn      = natural freq = {0:.4f} '.format(wn))
print('eta     = eta          = {0:.4f} '.format(eta))
print('roots   = roots        = \n',r)


# INSERT PRESSURE SENSOR CODE STARTUP HERE

# INSERT MOTOR STARTUP CODE HERE
keyhandle, NodeID = init_maxon()
 

# IMU STARTUP
# function call
yaw0,imu = init_imu()
yawOld=0
print('Yaw0:',yaw0)
print('YawOld:',yawOld)
print('IMU Port:',imu)


##### MAIN CONTROL LOOP ####
k=1
t = np.empty(maxiter)
theta = np.empty(maxiter)
vel = np.empty(maxiter)
acc = np.empty(maxiter)
Omega = np.empty(maxiter)
err = np.empty(maxiter)
errdot = np.empty(maxiter)
ecumul = np.empty(maxiter)
u = np.empty(maxiter)
umotor = np.empty(maxiter)

meantheta=0


t[0]=0
err[0]=-theta_d
tstart=time.time()
time.sleep(1.0E-16)
#while (k <= maxiter-1) and abs(meantheta - theta_d) > tol):
while (k < maxiter):
    t[k]=time.time() - tstart
    dt=t[k] - t[k-1]
#    print('Iteration #        = {0:4d} {1:4f} '.format(k,t[k]))
    time.sleep(1.0E-14)
    
# Get IMU data
    yaw,pitch,roll=imudata(imu,yaw0,yawOld)

    theta[k]=yaw
    vel[k] = (theta[k]) - theta[k-1]/dt
    acc[k]=(vel[k] - vel[k-1])/dt
#    Omega[k] = Motor1.ActualVelocity

    err[k]=(theta[k] - theta_d)
    errdot[k]=(err[k] - err[k-1])/dt
    ecumul[k]=ecumul[k-1] + err[k]*dt
    
# NOT CONVERTED ########################    
          
    if abs(err[k]) < 5:
        print("Reduce Acceleration - 10")
        acceleration=acceleration0*0.01
    elif abs(err[k]) < 10:
        print("Reduce Acceleration - 5\n")
        acceleration=acceleration0*0.1
    elif abs(err[k]) < 15:
        print("Reduce Acceleration - 5\n")
        acceleration=acceleration0*0.5
    else:
        acceleration=acceleration0;

    kp=kp0;kd=kd0;ki=ki0;    

    # New Algorithm Using Lagrangian Definitions (PID Controller)
    u[k] = (I/J)*(kp*err[k] + kd*errdot[k] + ki*ecumul[k]); 

#   printf("%5.0i    %4.3f  %10.2f  %10.2f %10.2f %10.2f %10.2f %10.2f  ,k,t(k), theta(k), meantheta, err(k), errdot(k), ecumul(k), u(k))
#    print(k,t[k], theta[k], meantheta, err[k], errdot[k], ecumul[k], u[k])
    print(k,t[k], dt, theta[k], theta[k-1], vel[k])

    
    umotor[k]=u[k];
    if abs(u[k]) > umax:
        umotor[k]=np.sign(u[k])*umax;
        
   # Drive the Motor
    if (k % 10 == 0):
#        print("Driving motor with umotor = " % umotor[k])
        print("Driving motor with umotor = {0:4d} {1:4f} ".format(k,umotor[k]))
        Omega[k]=MotionInVelocity(keyhandle, NodeID,umotor[k],acceleration);

    if (k > 10):
        meantheta=np.mean(theta[k-10:k])

    yawOld=theta[k]
    k=k+1

print("End of MAIN LOOP")    

#Hack Omega
#Omega=umotor*1.05

plotsloshy(t, theta, theta_d, u, umotor, umax, Omega, vel, acc, err, errdot, ecumul)
plt.show()

close_maxon(keyhandle,NodeID)

print("End of CODE")  




 