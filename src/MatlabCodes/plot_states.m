close all;
clear all;
clc;

f_traj =  fopen('../../records/traj_log.txt','r');

if f_traj == -1
    error('File traj_log.txt could not be opened, check name or path.')
end

traj_line= fgetl(f_traj);
virtual_traj = [];

while ischar(traj_line)
    % 1 1425787309.908592 28.209999 0.206472 0.375029 4.099180 0.274586 8.698499
    % st_current.speed st_current.yaw st_current.pitch accel_xyz.ax accel_xyz.ay accel_xyz.az
   log_traj = textscan(traj_line,'%d %f %f %f %f %f %f %f');
   seq= log_traj{1}
   t= log_traj{2};  
   speed= log_traj{3};
   yaw= log_traj{4};
   pitch= log_traj{5};
   ax= log_traj{6};
   ay= log_traj{7};
   az= log_traj{8};
   if seq > 1
      virtual_traj = [ virtual_traj; [t,speed,yaw,pitch,ax,ay,az] ];
   end 
   traj_line= fgetl(f_traj);
end

figure;
% hold on;
% axis auto;
% grid on;
plot(virtual_traj(:,1),virtual_traj(:,2),'+-');

% figure;
% plot(virtual_traj(:,1),virtual_traj(:,3),'-');

figure;
plot(virtual_traj(:,1),virtual_traj(:,4),'sq-');

figure;
plot(virtual_traj(:,1),virtual_traj(:,5),'r-');
hold on;
plot(virtual_traj(:,1),virtual_traj(:,6),'g-');
plot(virtual_traj(:,1),virtual_traj(:,7),'b-');
legend('ax','ay','az');