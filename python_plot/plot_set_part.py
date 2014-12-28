import numpy as np
import matplotlib.pyplot as plt
import math

def x1(the,t):
    global omiga,v,rho,r
    return -rho*(1-np.cos(the) )+(v*t+rho*the+r)*np.sin(the)

def y1(the,t):
    global omiga,v,rho,r
    return -rho*np.sin(the)+(v*t+rho*the+r)*np.cos(the)

def x2(the,t):
    global omiga,v,rho,r
    return rho*(1-np.cos(the))+(v*t-rho*the+r)*np.sin(the)

def y2(the,t):
    global omiga,v,rho,r
    return rho*np.sin(the)+(v*t-rho*the+r)*np.cos(the)

def x3(the,t):
    global up_the,v,rho,r
    return -rho*(1-np.cos(up_the)) - (v*t-rho*up_the)*np.sin(up_the) + r*np.sin(the)

def y3(the,t):
    global up_the,v,rho,r
    return rho*np.sin(up_the) + (v*t-rho*up_the)*np.cos(up_the) + r*np.cos(the)

def x4(the,t):
    global up_the,v,rho,r
    return rho*(1-np.cos(up_the)) + (v*t-rho*up_the)*np.sin(up_the) + r*np.sin(the)

def y4(the,t):
    global up_the,v,rho,r
    return rho*np.sin(up_the) + (v*t-rho*up_the)*np.cos(up_the) + r*np.cos(the)

def x(the,t):
    assert (-np.pi <= the <= np.pi), 'Theta only between -pi and pi'
    global omiga, up_the
    if up_the <= np.pi:
       if -up_the <= the <=0:
          return x1(the,t)
       elif 0<= the <= up_the:
          return x2(the,t)
       elif -np.pi <= the <= -up_the:
          return x3(the,t)
       #elif the >= omiga*t and the <= np.pi:
       else:
          return x4(the,t)
    else:
       if -np.pi <= the <= 0:
          return x1(the,t)
       #elif the <= np.pi and the >= 0: 
       else:
          return x2(the,t)

def y(the,t):
    assert (-np.pi<= the <= np.pi), 'Theta only between -pi and pi'
    global omiga, up_the
    if up_the <= np.pi:
        if -up_the <= the <=0:
           return y1(the,t)
        elif 0<= the <= up_the:
           return y2(the,t)
        elif -np.pi<= the <= -up_the:
           return y3(the,t)
        #elif the >= omiga*t and the <= np.pi:
        else:
           return y4(the,t)
    else:
       if -np.pi<= the <= 0:
          return y1(the,t)
       #elif the <= np.pi and the >= 0: 
       else:
          return y2(the,t)

xfun = np.vectorize(x)
yfun = np.vectorize(y)

r=300.
t=30.
v=120.
omiga= 3./180*np.pi
rho = v/omiga
up_the = np.pi/16

n_theta= 1001;
d_theta= 2*np.pi/(n_theta-1)
theta= np.arange(-np.pi, np.pi, d_theta)
#theta= np.linspace(-up_, up_the, num=n_theta)

x_set = xfun(theta,t)
y_set = yfun(theta,t)

xl= x(-np.pi,t)
yl= y(-np.pi,t)
xr= x(np.pi,t)
yr= y(np.pi,t)

plt.plot(x_set,y_set)
plt.plot([xl,xr],[yl,yr])

'''
n1= 20
#up_the = np.pi/2
theta1= np.linspace(-np.pi,np.pi,num=n1)
x_dis = xfun(theta1,t)
y_dis = yfun(theta1,t)
for i in range(0,theta1.size-1):
    plt.plot([ x_dis[i],x_dis[i+1] ],[y_dis[i],y_dis[i+1] ])
'''
v = 130.
rho = v/omiga

n_theta= 1001;
d_theta= 2*np.pi/(n_theta-1)
theta= np.arange(-np.pi, np.pi, d_theta)
#theta= np.linspace(-up_, up_the, num=n_theta)

x_set = xfun(theta,t)
y_set = yfun(theta,t)

xl= x(-np.pi,t)
yl= y(-np.pi,t)
xr= x(np.pi,t)
yr= y(np.pi,t)

plt.plot(x_set,y_set)
plt.plot([xl,xr],[yl,yr])

plt.show()
