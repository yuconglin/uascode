import numpy as np
import matplotlib.pyplot as plt

def x1(the,t):
    global omiga,v,rho,r
    return -rho*(1-cos(the) )+(v*t+rho*the+r)*sin(the)

def y1(the,t):
    global omiga,v,rho,r
    return -rho*sin(the)+(v*t+rho*the+r)*cos(the)

def x2(the,t):
    global omiga,v,rho,r
    return rho(1-cos(the))+(v*t-rho*the+r)*sin(the)

def y2(the,t):
    global omiga,v,rho,r
    return rho*sin(the)+(v*t-rho*the+r)*cos(the)

def x3(the,t):
    global omiga,v,rho,r
    return -rho*(1-cos(omiga*t))+r*sin(the)

def y3(the,t):
    global omiga,v,rho,r
    return rho*sin(omiga*t)+r*cos(the)

def x4(the,t):
    global omiga,v,rho,r
    return rho*(1-cos(omiga*t))+r*sin(the)

def y4(the,t):
    global omiga,v,rho,r
    return rho*sin(omiga*t)+r*cos(the)

def x(the,t):
    '''global omiga,v,rho'''
    if omiga*t <= np.pi:
       if the >= max(-omiga*t,-np.pi) & the <=0:
    else:

t = 10
omiga= 1.5/180*np.pi
theta= np.arange(-np.pi,0.01,np.pi)

