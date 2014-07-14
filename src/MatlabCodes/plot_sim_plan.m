close all;
clear all;
clc;

%load planned path
f_traj =fopen('../../bin/fs_state.txt','r');

if f_traj == -1
    error('File fs_state.txt could not be opened, check name or path.')
end
traj_line= fgetl(f_traj);
virtual_traj = [];
count1=0;
while ischar(traj_line)
    %26446 416307.2 3699693 33.43336 -111.9003 1003.729 29.53804 104.5282 7.639437 1.037654 -3.730131 2.088005 7771.462 
   log_traj = textscan(traj_line,'%f %f %f %f %f %f %f %f %f %f %f %f %f');
   lat = log_traj{4};
   lon = log_traj{5};
   
   if count1== 0
     t0= log_traj{1};    
   end
   
   z = log_traj{6};
   t = log_traj{1}- t0;
   v = log_traj{7};
   x = log_traj{2};
   y = log_traj{3};
   yaw = log_traj{8}+ 360;
   if yaw > 360
      yaw= yaw-360;
   end
   
   pitch = log_traj{9};
%    if yaw > 180
%        yaw= yaw -180;
%    elseif yaw < -180
%        yaw= yaw +180;
%    end
   
   virtual_traj = [ virtual_traj; [t,x,y,z,v,yaw,pitch] ];
   traj_line= fgetl(f_traj);
   count1= count1+1;
end

%load actual sitl path
s_traj =fopen('../../bin/sitl_state1.txt','r');

if s_traj == -1
    error('File sitl_state.txt could not be opened, check name or path.')
end
sim_line= fgetl(s_traj);
sim_traj = [];
count2= 0;
while ischar(sim_line)
%1405050339.43151 -111.93368 33.42466 624.63000 26.40076 413198.24766 3698756.30170 254.00000 -0.04974 -0.06831 -1.84719 262.00000 153.00000 -603.00000
   log_traj = textscan(sim_line,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
   lat_s = log_traj{3};
   lon_s = log_traj{2};
   
   if count2== 0
     t0= log_traj{1};    
   end
   
   z_s = log_traj{4};
   t_s = log_traj{1}-t0;
   v_s = log_traj{5};
   x_s = log_traj{6};
   y_s = log_traj{7};
   
   hd_s = log_traj{8};
   pitch_s = log_traj{10};
   yaw_s = log_traj{11};
%    if hd_s > 180
%        hd_s= hd_s -180;
%    elseif hd_s < -180
%        hd_s= hd_s +180;
%    end
   
   sim_traj = [ sim_traj; [t_s,x_s,y_s,z_s,v_s,hd_s,pitch_s,yaw_s] ];
   sim_line= fgetl(s_traj);
   count2= count2+1;
end

%time stamp matching
t_array= virtual_traj(:,1);
ts_array= sim_traj(:,1);

for j=1:length(t_array)
  for i=1:length(ts_array)-1
     if ts_array(i)<= t_array(j) && ts_array(i+1)>= t_array(j)
         dis(j)= sqrt( (virtual_traj(j,2)-sim_traj(i,2) )^2 + (virtual_traj(j,3)-sim_traj(i,3) )^2 );
         break;
     end
  end
end

figure;
subplot(2,2,1);
hold on;
plot( sim_traj(:,1), sim_traj(:,2) );
plot( virtual_traj(:,1), virtual_traj(:,2),'r+-');

subplot(2,2,2);
hold on;
plot( sim_traj(:,1), sim_traj(:,3) );
plot( virtual_traj(:,1), virtual_traj(:,3), 'r+-' );

subplot(2,2,3);
hold on;
plot( sim_traj(:,1), sim_traj(:,4) );
plot( virtual_traj(:,1), virtual_traj(:,4), 'r+-' );

subplot(2,2,4);
hold on;
plot( sim_traj(:,1), sim_traj(:,5) );
plot( virtual_traj(:,1), virtual_traj(:,5), 'r+-' );

figure;
subplot(2,2,1);
hold on;
plot( sim_traj(:,1), sim_traj(:,6) );
plot( virtual_traj(:,1), virtual_traj(:,6), 'r+-' ); 

subplot(2,2,2);
hold on;
plot( t_array(1:length(dis)), dis );

subplot(2,2,3);
hold on;
plot( sim_traj(:,1), sim_traj(:,7)*180/pi );
plot( virtual_traj(:,1), virtual_traj(:,7), 'r+-' );

subplot(2,2,4);
hold on;
plot( sim_traj(:,1), sim_traj(:,8)*180/pi );
plot( virtual_traj(:,1), virtual_traj(:,6), 'r+-' );

figure;
hold on;
plot3(sim_traj(:,2),sim_traj(:,3),sim_traj(:,4) );
plot3(virtual_traj(:,2),virtual_traj(:,3),virtual_traj(:,4),'r+' );