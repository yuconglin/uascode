close all;
clear all;
clc;

time= 30;
MATDIS= zeros(3,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%load adsb logged (extropolated data)
f = fullfile('../../recordsHIL/','scaled_obstacle_10734770.txt');
f_obss =fopen(f,'r');

if f_obss == -1
  error('obstacle file could not be opened, check name or path.')
end

obss_vec1 = [];
obss_line= fgetl(f_obss);

while ischar(obss_line)
   log_obss = textscan(obss_line,'%d %f %f %f %f %f %f %f %f %f');
   address= log_obss{1};
   x= log_obss{2};
   y= log_obss{3};
   z= log_obss{4}; 
   t= log_obss{8};
   obss_vec1 = [obss_vec1;[t,x,y,z] ];
   obss_line= fgetl(f_obss);
end
%plot
% figure;
% plot3(obss_vec1(:,2),obss_vec1(:,3),obss_vec1(:,4),'r+' );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = fullfile('../../recordsHIL/','scaled_obstacle_10934723.txt');
f_obss =fopen(f,'r');

if f_obss == -1
  error('obstacle file could not be opened, check name or path.')
end

obss_vec2 = [];
obss_line= fgetl(f_obss);

while ischar(obss_line)
   log_obss = textscan(obss_line,'%d %f %f %f %f %f %f %f %f %f');
   address= log_obss{1};
   x= log_obss{2};
   y= log_obss{3};
   z= log_obss{4}; 
   t= log_obss{8};
   obss_vec2 = [obss_vec2;[t,x,y,z] ];
   obss_line= fgetl(f_obss);
end
%plot
% figure;
% plot3(obss_vec2(:,2),obss_vec2(:,3),obss_vec2(:,4),'r+' );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = fullfile('../../recordsHIL/','scaled_obstacle_10942331.txt');
f_obss =fopen(f,'r');

if f_obss == -1
  error('obstacle file could not be opened, check name or path.')
end

obss_vec3 = [];
obss_line= fgetl(f_obss);

while ischar(obss_line)
   log_obss = textscan(obss_line,'%d %f %f %f %f %f %f %f %f %f');
   address= log_obss{1};
   x= log_obss{2};
   y= log_obss{3};
   z= log_obss{4}; 
   t= log_obss{8};
   obss_vec3 = [obss_vec3;[t,x,y,z] ];
   obss_line= fgetl(f_obss);
end
%plot
% figure;
% plot3(obss_vec3(:,2),obss_vec3(:,3),obss_vec3(:,4),'r+' );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=2:length(virtual_traj)
    if(virtual_traj(i-1,1)-virtual_traj(1,1) < time && virtual_traj(i,1)-virtual_traj(1,1) >= time)
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

for i=2:length(obss_vec1)
    if(obss_vec1(i-1,1)-obss_vec1(1,1) < time && obss_vec1(i,1)-obss_vec1(1,1) >= time)
       x1= obss_vec1(i,2);
       y1= obss_vec1(i,3);
       z1= obss_vec1(i,4);
       break;
    end
end
if i== length(obss_vec1)
    x1= obss_vec1(i,2);
    y1= obss_vec1(i,3);
    z1= obss_vec1(i,4);
end


for i=2:length(obss_vec2)
    if(obss_vec2(i-1,1)-obss_vec2(1,1) < time && obss_vec2(i,1)-obss_vec2(1,1) >= time)
       x2= obss_vec2(i,2);
       y2= obss_vec2(i,3);
       z2= obss_vec2(i,4);
       break;
    end 
end
if i== length(obss_vec2)
    x2= obss_vec2(i,2);
    y2= obss_vec2(i,3);
    z2= obss_vec2(i,4);
end

for i=2:length(obss_vec3)
    if(obss_vec3(i-1,1)-obss_vec3(1,1) < time && obss_vec3(i,1)-obss_vec3(1,1) >= time)
       x3= obss_vec3(i,2);
       y3= obss_vec3(i,3);
       z3= obss_vec3(i,4);
       break;
    end
end
if i== length(obss_vec3)
    x3= obss_vec2(i,2);
    y3= obss_vec2(i,3);
    z3= obss_vec2(i,4);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MATDIS(1,:) = [x_traj-x1, y_traj-y1, z_traj-z1, 0];
MATDIS(2,:) = [x_traj-x2, y_traj-y2, z_traj-z2, 0];
MATDIS(3,:) = [x_traj-x3, y_traj-y3, z_traj-z3, 0];
