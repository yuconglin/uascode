close all
f_traj= fopen('../../bin/path_log.txt','r');

if f_traj == -1
  error('file path_log.txt could not be opened, check name or path.')
end

traj_line= fgetl(f_traj);
traj = []

while ischar(traj_line)
  %27 404090.7 3696013 33.39915 -112.0313 964.3626 35 -71.44792 -4.5 -1.286892 -3.214957 0.304382
  log_traj = textscan(traj_line,'%f %f %f %f %f %f %f %f %f %f %f %f');
  x = log_traj{2};
  y = log_traj{3};
  z = log_traj{6};
  t = log_traj{1};
  traj = [traj;[x,y,z,t] ];
  traj_line= fgetl(f_traj);
end

f1= fopen('../../bin/out_points.txt','r');

if f1 == -1
  error('file out_points.txt could not be opened');
end
f1_line= fgetl(f1);
point = [];

while ischar(f1_line)
  log_f1 = textscan(f1_line,'%f %f %f');
  x = log_f1{1};
  y = log_f1{2};
  z = log_f1{3};
  point = [point;[x,y,z] ];
  f1_line= fgetl(f1);
end

figure;
hold on;
axis auto;
grid on;
title('one full path');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
plot3( traj(:,1),traj(:,2),traj(:,3), 'r*' );
%plot3(422126, 3.69454e+06, 1010, 'b*' );
%plot3(421651,3.6931e+06,1010,'b*');
plot3( point(:,1),point(:,2),point(:,3), 'b*');

view(3);
