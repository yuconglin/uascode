close all;
clear all;
clc;

obs_files =[
%'obstacle_10734770.txt   ';
%'obstacle_10934723.txt   ';
% 'obstacle_10942331.txt   ';
'tt_obstacle_10734770.txt';
%'tt_obstacle_10934723.txt';
'tt_obstacle_10942331.txt';
];

cells = cellstr(obs_files);

for i=1:length(cells)
%for i=1:1
   %close all;
   
   f = fullfile('../../records/',cells{i});
   f_obss =fopen(f,'r');
   
   if f_obss == -1
      error('obstacle file could not be opened, check name or path.')
   end
    
   obss_vec = [];
   data_t = [];
   obss_line= fgetl(f_obss);
   
   while ischar(obss_line)
       log_obss = textscan(obss_line,'%d %f %f %f %f %f %f %f %f %f');
       address= log_obss{1};
       x= log_obss{2};
       y= log_obss{3};
       z= log_obss{4}; 
       hd= log_obss{5};
       speed= log_obss{6};
       v_vert = log_obss{7};
       t= log_obss{8};
       obss_vec = [obss_vec;[x,y,z] ];
       data_t = [data_t;[t,speed,hd,v_vert] ];
       obss_line= fgetl(f_obss);
   end
   
%    figure;
%    h=plot3(obss_vec(:,1),obss_vec(:,2),obss_vec(:,3),'r+');
%    view(2);

%    figure;
%    plot(data_t(:,1),data_t(:,2) );
   
   figure;
   plot(data_t(:,1),data_t(:,4) );
   
   lm = length(data_t);
   hd_pre = data_t(1:lm-1,3);
   hd = data_t(2:lm,3);
   
%    figure;
%    plot(data_t(2:lm,1), hd-hd_pre, 'k+');
   
end