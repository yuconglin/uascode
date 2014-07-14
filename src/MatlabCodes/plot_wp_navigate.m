close all;
f_traj =fopen('../../bin/fs_state.txt','r');

if f_traj == -1
    error('File fs_state.txt could not be opened, check name or path.')
end
traj_line= fgetl(f_traj);
virtual_traj = [];
while ischar(traj_line)
    %26446 416307.2 3699693 33.43336 -111.9003 1003.729 29.53804 104.5282 7.639437 1.037654 -3.730131 2.088005 7771.462 
   log_traj = textscan(traj_line,'%f %f %f %f %f %f %f %f %f %f %f %f %f');
   x = log_traj{2};
   y = log_traj{3};
   z = log_traj{6};
   t = log_traj{1};
   virtual_traj = [ virtual_traj; [x,y,z,t] ];
   traj_line= fgetl(f_traj);
end

figure;
hold on;
axis auto;
grid on;
title('fs_state');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
plot3( virtual_traj(:,1),virtual_traj(:,2),virtual_traj(:,3), 'r*' );
%plot3(422126, 3.69454e+06, 1010, 'b*' );

view(3);
