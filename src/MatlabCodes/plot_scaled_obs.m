close all;
clear all;
clc;

f_obss= fopen('../../recordsHIL/simuObsScaled.txt');
%f_obss = fopen('../../data/20140924-212359obs.txt');
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
%   r = log_traj{9};
%   hr = log_traj{10};
  
  has_obs = [t,x,y,z,speed,v_vert];
  
  if(address == addrs(1) )
      obs1= [obs1; has_obs];
  elseif(address == addrs(2) )
      obs2= [obs2; has_obs];
  else
      obs3= [obs3; has_obs];     
  end
  
  obss_line= fgetl(f_obss);
  
end

figure;
hold on;
axis auto;
grid on;
title('scaled obstacle');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
plot3( obs1(:,2),obs1(:,3),obs1(:,4), 'r*' );

view(3);

figure;
axis auto;
grid on;
title('speed');
plot( obs1(:,1), obs1(:,5), 'k*' );

figure;
axis auto;
grid on;
title('vertical speed');
plot( obs1(:,1), obs1(:,6), 'b*' );


