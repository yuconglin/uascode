close all;
clear all;
clc;

obs_files =[
'whole_obstacle_10947687.txt';
'whole_obstacle_11214846.txt';
'whole_obstacle_11349796.txt';
'whole_obstacle_4196010.txt '
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
   obss_line= fgetl(f_obss);
   
   while ischar(obss_line)
       log_obss = textscan(obss_line,'%d %f %f %f %f %f %f %f %f %f');
       address= log_obss{1};
       x= log_obss{2};
       y= log_obss{3};
       z= log_obss{4}; 
       obss_vec = [obss_vec;[x,y,z] ];
       obss_line= fgetl(f_obss);
   end
   
   figure;
   h=plot3(obss_vec(:,1),obss_vec(:,2),obss_vec(:,3),'r+');
%    f_png = fullfile('../../pngs/',strcat(int2str(address),'.png') );
%    saveas(h, f_png, 'png');
   
end