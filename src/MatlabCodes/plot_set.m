close all;
clear all;
clc;

%f_traj = fopen('/home/yucong/catkin_ws/devel/lib/uascode/set20.txt','r');
f_traj = fopen('/home/yucong/catkin_ws/src/uascode/real_set.txt','r');
%f_traj = fopen('/home/yucong/catkin_ws/devel/lib/uascode/real_set.txt','r');

if f_traj == -1
    error('File set.txt could not be opened, check name or path.')
end
line1 = fgetl(f_traj);
set_points = [];

while ischar(line1)
    log_traj = textscan(line1,'%f %f');
    x = log_traj{1};
    y = log_traj{2};
    set_points = [ set_points; [x,y] ];
    line1= fgetl(f_traj);
end

figure
axis equal;
%axis([-300 300 0 800])
plot( set_points(:,1), set_points(:,2), '-r+' );
%line( [100,100+50*cosd(30)], [34,34+50*sind(30)] ); 
hold on;

% f_pts = fopen('/home/yucong/catkin_ws/devel/lib/uascode/ran_points.txt','r');
% line2 = fgetl(f_pts);
% 
% while ischar(line2)
%     log_pts = textscan(line2,'%f %f %d');
%     xp = log_pts{1};
%     yp = log_pts{2};
%     in = log_pts{3};
%     if in == 1
%        plot( xp, yp, 'r*');    
%     else
%        plot( xp, yp, 'g*' );
%     end
%     line2 = fgetl(f_pts);
% end 