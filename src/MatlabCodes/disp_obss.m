close all;
clear all;
clc;

% obs_files= [
% 'obstacle_10677635.txt';
% 'obstacle_10719631.txt';
% 'obstacle_10734770.txt';
% 'obstacle_10922114.txt';
% 'obstacle_10931269.txt';
% 'obstacle_10934723.txt';
% 'obstacle_10942331.txt';
% 'obstacle_10944834.txt';
% 'obstacle_10973615.txt';
% 'obstacle_11019675.txt';
% 'obstacle_11211267.txt';
% 'obstacle_11229295.txt';
% 'obstacle_11372509.txt';
% 'obstacle_13631493.txt';
% 'obstacle_13631526.txt';
% 'obstacle_13631529.txt';
% 'obstacle_13631697.txt';
% 'obstacle_13633837.txt';
% 'obstacle_13634196.txt';
% 'obstacle_13635392.txt';
% 'obstacle_13635526.txt';
% 'obstacle_13635562.txt'
% ];

obs_files = [
'obstacle_10734770.txt'
%'obstacle_10934723.txt'
%'obstacle_10942331.txt'
];

cells = cellstr(obs_files);
%load each file; plot and save into png
for i=1:length(cells)
%for i=1:1
   close all;
   
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
       speed= log_obss{6};
       vert= log_obss{7};
       t= log_obss{8};
       
       obss_vec = [obss_vec;[x,y,z] ];
       data_t = [data_t;[t,speed,vert] ];
       
       obss_line= fgetl(f_obss);
   end
   
   figure;
   h=plot3(obss_vec(:,1),obss_vec(:,2),obss_vec(:,3),'r+');
%    f_png = fullfile('../../pngs/',strcat(int2str(address),'.png') );
%    saveas(h, f_png, 'png');
   
   figure;
   plot( data_t(:,1), obss_vec(:,1), 'y+' );
   title('x');

   figure;
   plot( data_t(:,1), data_t(:,2), '-b+' );
   title('speed');
   
   figure;
   plot( data_t(:,1), data_t(:,3), '-k+' );
   title('v_speed');
   
end