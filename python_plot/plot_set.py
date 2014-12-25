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
    global omiga,v,rho,r
    return -rho*(1-np.cos(omiga*t))+r*np.sin(the)

def y3(the,t):
    global omiga,v,rho,r
    return rho*np.sin(omiga*t)+r*np.cos(the)

def x4(the,t):
    global omiga,v,rho,r
    return rho*(1-np.cos(omiga*t))+r*np.sin(the)

def y4(the,t):
    global omiga,v,rho,r
    return rho*np.sin(omiga*t)+r*np.cos(the)

def x(the,t):
    assert (-np.pi <= the <= np.pi), 'Theta only between -pi and pi'
    global omiga
    if omiga*t <= np.pi:
       if -omiga*t <= the <=0:
          return x1(the,t)
       elif 0<= the <= omiga*t:
          return x2(the,t)
       elif -np.pi <= the <= -omiga*t:
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
    global omiga
    if omiga*t <= np.pi:
        if -omiga*t <= the <=0:
           return y1(the,t)
        elif 0<= the <= omiga*t:
           return y2(the,t)
        elif -np.pi<= the <= -omiga*t:
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

def PolyArea(X,Y):
    numPoints = X.size
    print "numPoints:%d" % numPoints
    area = 0.;         #Accumulates area in the loop
    j = numPoints-1;  # The last vertex is the 'previous' one to the first

    for i in range(0,numPoints):
        #print "%f" % (X[j]+X[i]) * (Y[j]-Y[i])
        area = area + (X[j]+X[i]) * (Y[j]-Y[i]) 
        j = i  #j is previous vertex to i
    
    return area/2


xfun = np.vectorize(x)
yfun = np.vectorize(y)
'''
t = 10
omiga= 1.5/180*np.pi
v= 60
rho= v/omiga
r= 100
'''

r=5.
t=20.
v=1.
rho=10.
omiga= v/rho

n_theta= 1001;
d_theta= 2*np.pi/(n_theta-1)
#theta= np.arange(-np.pi, np.pi, d_theta)
theta= np.linspace(-np.pi, np.pi, num=n_theta)
area1=0.
for i in range(0,theta.size-1):
    area1+= x_dot(theta[i],t)*y(theta[i],t)*d_theta
    
sq_area= (x(np.pi,t)-x(-np.pi,t) )*y(np.pi,t)
area1-= sq_area
print("integrated area:%f" % area1)

x_set = xfun(theta,t)
y_set = yfun(theta,t)

xl= x(-np.pi,t)
yl= y(-np.pi,t)
xr= x(np.pi,t)
yr= y(np.pi,t)

plt.plot(x_set,y_set)
plt.plot([xl,xr],[yl,yr])
'''
v=1.5
omiga= v/rho
x_set = xfun(theta,t)
y_set = yfun(theta,t)

xl= x(-np.pi,t)
yl= y(-np.pi,t)
xr= x(np.pi,t)
yr= y(np.pi,t)

plt.plot(x_set,y_set)
plt.plot([xl,xr],[yl,yr])
'''
n1= 10
theta1= np.linspace(-np.pi,np.pi,num=n1)
x_dis = xfun(theta1,t)
y_dis = yfun(theta1,t)
for i in range(0,theta1.size-1):
    plt.plot([ x_dis[i],x_dis[i+1] ],[y_dis[i],y_dis[i+1] ])

print "discrete area:%f" % PolyArea(x_dis,y_dis)
print "ratio:%f" % (PolyArea(x_dis,y_dis)/area1)

plt.show()
