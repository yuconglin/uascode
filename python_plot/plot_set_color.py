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

colors = ['b', 'y', 'r', 'k', 'c']

n_theta = 1001

theta_l1 = np.linspace(-up_the,0,num=n_theta)
theta_r1 = np.linspace(0,up_the,num=n_theta)
theta_l2 = np.linspace(-np.pi,-up_the,num=n_theta)
theta_r2 = np.linspace(up_the,np.pi,num=n_theta)

theta2 = np.concatenate([theta_l2,theta_r2])

x_l1 = xfun(theta_l1,t)
y_l1 = yfun(theta_l1,t)

x_r1 = xfun(theta_r1,t)
y_r1 = yfun(theta_r1,t)

x_l2 = xfun(theta_l2,t)
y_l2 = yfun(theta_l2,t)

x_r2 = xfun(theta_r2,t)
y_r2 = yfun(theta_r2,t)
'''
xs2 = xfun(theta2,t)
ys2 = yfun(theta2,t)
'''
xl= x(-np.pi,t)
yl= y(-np.pi,t)
xr= x(np.pi,t)
yr= y(np.pi,t)

plt.plot(x_l1,y_l1,color=colors[0],label='$S_1$')
plt.plot(x_r1,y_r1,color=colors[1],label='$S_2$')
plt.plot(x_l2,y_l2,color=colors[2],label='$S_3$')
plt.plot(x_r2,y_r2,color=colors[3],label='$S_4$')
plt.plot([xl,xr],[yl,yr],color=colors[4],label='$S_5$')

plt.xlabel('x (m)')
plt.ylabel('y (m)')

plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=5, mode="expand", borderaxespad=0.)
#plt.axis('equal')
plt.ylim(3000, 4000)
#plt.gca().set_aspect('equal', adjustable='box')
plt.show()
