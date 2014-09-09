close all;
clear all;
clc;

obs_files= [
'obstacle_10093017.txt';
'obstacle_10093305.txt';
'obstacle_10093522.txt';
'obstacle_10094336.txt';
'obstacle_10095082.txt';
'obstacle_10095604.txt';
'obstacle_10095728.txt';
'obstacle_10096229.txt';
'obstacle_10537990.txt';
'obstacle_10538941.txt';
'obstacle_10782757.txt';
'obstacle_10811093.txt';
'obstacle_10923065.txt';
'obstacle_10947687.txt';
'obstacle_10966007.txt';
'obstacle_10966958.txt';
'obstacle_11003128.txt';
'obstacle_11192443.txt';
'obstacle_11214846.txt';
'obstacle_11245404.txt';
'obstacle_11250923.txt';
'obstacle_11317733.txt';
'obstacle_11349796.txt';
'obstacle_11355324.txt';
'obstacle_11357226.txt';
'obstacle_13631554.txt';
'obstacle_13631561.txt';
'obstacle_13632248.txt';
'obstacle_13632341.txt';
'obstacle_13632385.txt';
'obstacle_13632416.txt';
'obstacle_13632497.txt';
'obstacle_13632600.txt';
'obstacle_13632687.txt';
'obstacle_13632795.txt';
'obstacle_13632861.txt';
'obstacle_13632899.txt';
'obstacle_13632902.txt';
'obstacle_13633187.txt';
'obstacle_13633220.txt';
'obstacle_13633251.txt';
'obstacle_13633869.txt';
'obstacle_13634120.txt';
'obstacle_13634215.txt';
'obstacle_13634259.txt';
'obstacle_13634314.txt';
'obstacle_13634331.txt';
'obstacle_13634535.txt';
'obstacle_13634558.txt';
'obstacle_13634682.txt';
'obstacle_13634861.txt';
'obstacle_13634959.txt';
'obstacle_13634963.txt';
'obstacle_13635000.txt';
'obstacle_13635136.txt';
'obstacle_13635274.txt';
'obstacle_13635367.txt';
'obstacle_13635461.txt';
'obstacle_13635468.txt';
'obstacle_4196010.txt '
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
   f_png = fullfile('../../pngs/',strcat(int2str(address),'.png') );
   saveas(h, f_png, 'png');
   
end