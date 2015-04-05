close all;
clear all;
clc;

time= 30;
MATDIS= zeros(3,4);

f_traj =fopen('../../recordsHIL/traj_log.txt','r');

if f_traj == -1
    error('File traj_log.txt could not be opened, check name or path.')
end

traj_line= fgetl(f_traj);
virtual_traj = [];

while ischar(traj_line)
   %1 1428105584.929208 33.440809 -112.025585 404671.344711 3700626.957951 729.358000 13.853501 1.402059 0.116651
   
   log_traj = textscan(traj_line,'%d %f %f %f %f %f %f %f %f %f');
   wp_num = log_traj{1};
   t= log_traj{2};
   lat= log_traj{3};
   lon= log_traj{4};
   x= log_traj{5};
   y= log_traj{6};
   z= log_traj{7};
   speed= log_traj{8};
   yaw= log_traj{9};
   pitch= log_traj{10};
   
   virtual_traj = [ virtual_traj; [t,x,y,z] ];
   
   traj_line= fgetl(f_traj);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_obss= fopen('../../recordsHIL/simuObsScaled.txt');

if f_obss == -1
    error('File simuObsScaled.txt could not be opened, check name or path.')
end

obss_line= fgetl(f_obss);
obs1= [];
obs2= [];
obs3= [];
addrs = [11259136,11259137,11259138];

while ischar(obss_line)
  % 11259137 406094.60604366 3700204.69378400 10363.20000000 150.46875323 249.50555534 0.00000000 15846.00000000 640.08000000 152.40000000 
  log_obss = textscan(obss_line,'%d %f %f %f %f %f %f %f %f %f');
  address= log_obss{1};
  x= log_obss{2};
  y= log_obss{3};
  z= log_obss{4};
  hd= log_obss{5};
  speed= log_obss{6};
  v_vert= log_obss{7};
  t = log_obss{8};
  
  has_obs = [t,x,y,z];
  
  if(address == addrs(1) )
      obs1= [obs1; has_obs];
  elseif(address == addrs(2) )
      obs2= [obs2; has_obs];
  else
      obs3= [obs3; has_obs];     
  end
  
  obss_line= fgetl(f_obss);
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=2:length(virtual_traj)
    if(virtual_traj(i-1,1)-virtual_traj(1,1) < time && virtual_traj(i,1)-virtual_traj(1,1) >= time)
%        x_traj= virtual_traj(i,2);
%        y_traj= virtual_traj(i,3);
%        z_traj= virtual_traj(i,4);
       x1 = virtual_traj(i-1,2);
       x2 = virtual_traj(i,2);
       y1 = virtual_traj(i-1,3);
       y2 = virtual_traj(i,3);
       z1 = virtual_traj(i-1,4);
       z2 = virtual_traj(i,4);
       t1 = virtual_traj(i-1,1)-virtual_traj(1,1);
       t2 = virtual_traj(i,1)-virtual_traj(1,1);
       tc = time;
       c = (tc - t1)/(t2-t1);
       x_traj = c * ( x2 - x1 ) + x1;
       y_traj = c * ( y2 - y1 ) + y1;
       
       if ( z2 == z1 )
          z_traj = z2;    
       else 
          z_traj = c * ( z2 - z1 ) + z1;
       end
       
       break;
    end
end

for i=2:length(obs1)
    if(obs1(i-1,1)-obs1(1,1) < time && obs1(i,1)-obs1(1,1) >= time)
       x1= obs1(i,2);
       y1= obs1(i,3);
       z1= obs1(i,4);
       break;
    end
end
if i== length(obs1)
    x1= obs1(i,2);
    y1= obs1(i,3);
    z1= obs1(i,4);
end

for i=2:length(obs2)
    if(obs2(i-1,1)-obs2(1,1) < time && obs2(i,1)-obs2(1,1) >= time)
       x2= obs2(i,2);
       y2= obs2(i,3);
       z2= obs2(i,4);
       break;
    end
end
if i== length(obs2)
    x2= obs2(i,2);
    y2= obs2(i,3);
    z2= obs2(i,4);
end

for i=2:length(obs3)
    if(obs3(i-1,1)-obs3(1,1) < time && obs3(i,1)-obs3(1,1) >= time)
       x3= obs3(i,2);
       y3= obs3(i,3);
       z3= obs3(i,4);
       break;
    end
end
if i== length(obs3)
    x3= obs3(i,2);
    y3= obs3(i,3);
    z3= obs3(i,4);
end

MATDIS(1,:) = [x_traj-x1, y_traj-y1, z_traj-z1, 0];
MATDIS(2,:) = [x_traj-x2, y_traj-y2, z_traj-z2, 0];
MATDIS(3,:) = [x_traj-x3, y_traj-y3, z_traj-z3, 0];