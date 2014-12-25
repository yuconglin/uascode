import numpy as np
import matplotlib.pyplot as plt
#my own lib
from plot_set import x,y 

def x1_dot(the,t):
    global omiga,v,rho,r
    return (v*t+rho*the+r)*np.cos(the)

def x2_dot(the,t):
    global omiga,v,rho,r
    return (v*t-rho*the+r)*np.cos(the)

def x3_dot(the,t):
    global r
    return r*np.cos(the)

def x4_dot(the,t):
    global r
    return r*np.cos(the)

def x_dot(the,t):
    assert (-np.pi <= the <= np.pi), 'Theta only between -pi and pi'
    global omiga
    if omiga*t <= np.pi:
       if -omiga*t <= the <=0:
	  return x1_dot(the,t)
       elif 0<= the <= omiga*t:
	  return x2_dot(the,t)
       elif -np.pi <= the <= -omiga*t:
	  return x3_dot(the,t)
       #elif the >= omiga*t and the <= np.pi:
       else:
	  return x4_dot(the,t)
    else:
       if -np.pi <= the <= 0:
	  return x1_dot(the,t)
       #elif the <= np.pi and the >= 0: 
       else:
	  return x2_dot(the,t)

#the square area
sq_area= (x(np.pi,t)-x(-np.pi,t) )*y(np.pi,t)

d_theta= 0.001
theta= np.arange(-np.pi, np.pi, d_theta)
area1=0.
for i in range(0,theta.size):
    area1+= x_dot(theta[i],t)*y(theta[i],t)*d_theta
    
area1-= sq_area
print("integrated area:%f" % area1)
