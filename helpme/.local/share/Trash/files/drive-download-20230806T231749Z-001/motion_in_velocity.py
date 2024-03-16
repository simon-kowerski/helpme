# -*- coding: utf-8 -*-
"""
Created on Sat Aug  5 13:14:56 2023

@author: mfogel
"""

from ctypes import *

def MotionInVelocity(keyhandle, NodeID, umotor,acceleration):

    
    # Acceleration and Umotor are DWORD = unsigned int
    
    ret=0
    pErrorCode=c_uint()
    pDeviceErrorCode=c_uint()
    pVelocityIs=c_long()  #Look at ctypes library
    
    ret = epos.VCS_SetVelocityProfile(keyhandle, NodeID, acceleration, acceleration, byref(pErrorCode))
    #EXAMPLE
    #ret= epos.VCS_ActivateProfileVelocityMode(keyhandle, NodeID, byref(pErrorCode) )
    #EXAMPLE

    ret = epos.VCS_MoveWithVelocity(keyhandle, NodeID, acceleration, umotor, byref(pErrorCode))

    ret = epos.VCS_GetVelocityIs(keyhandle, NodeID, byref(pVelocityIs), byref(pErrorCode))

    #BOOL VCS_GetVelocityIs(HANDLE KeyHandle, WORD NodeId, long* pVelocityIs, DWORD* pErrorCode)

    print("Motor Velocity Is = " % pVelocityIs.value)
    
    #Omega = 1.999 
    Omega = pVelocityIs.value
    print("Omega = " % Omega)
    return Omega