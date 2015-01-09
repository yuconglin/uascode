close all;
clear all;
clc;

obs_files = [
'tt_obstacle_10734770.txt';
%'tt_obstacle_10934723.txt';
%'tt_obstacle_10942331.txt';
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
   
   figure;
   axis equal
   xc = 3.934e5;
   yc = 3.701e6;
   
   xs = 3.927e5;
   ys = 3.702e6;
   
   %h=plot3(obss_vec(:,1),obss_vec(:,2),obss_vec(:,3),'r+');
   plot(obss_vec(:,1),obss_vec(:,2),'r+');
   viscircles([xc,yc],2000,'EdgeColor','b','LineStyle','-.');
   viscircles([xs,ys],15,'LineWidth',14,'EdgeColor','k');
   
   xlabel('x(m)')
   ylabel('y(m)')
   
   %view(2);

%    figure;
%    plot( data_t(:,1), obss_vec(:,1), 'y+' );
%    title('x');
%    
%    figure;
%    plot( data_t(:,1), obss_vec(:,3), 'b+' );
%    title('z');
%    
%    figure;
%    plot(data_t(:,1),data_t(:,2) );
%    title('speed');
%    
%    figure;
%    plot(data_t(:,1),data_t(:,3) );
%    title('heading');
%    
%    figure;
%    plot(data_t(:,1),data_t(:,4) );
%    title('vert');
% %    
%    lm = length(data_t);
%    hd_pre = data_t(1:lm-1,3);
%    hd = data_t(2:lm,3);
%    
%    figure;
%    plot(data_t(2:lm,1), hd-hd_pre, 'k+');
%    hold on;
%    plot(data_t(:,1), data_t(:,3) );
%    title('hd');
   
end