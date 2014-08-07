close all;
clear all;
clc;

%to plot the whole trajectory and the distance between the obstacles and
%the aircraft

f_traj =fopen('../../records/traj_log0.txt','r');

if f_traj == -1
    error('File traj_log.txt could not be opened, check name or path.')
end

traj_line= fgetl(f_traj);
virtual_traj = [];

while ischar(traj_line)
    %1406173247.931275 33.427010 -111.927269 413796.566721 3699011.176642 730.559998 25.079872 0.350438 0.324690 
   log_traj = textscan(traj_line,'%f %f %f %f %f %f %f %f %f');
   t= log_traj{1};
   lat= log_traj{2};
   lon= log_traj{3};
   x= log_traj{4};
   y= log_traj{5};
   z= log_traj{6};
   speed= log_traj{7};
   yaw= log_traj{8};
   pitch= log_traj{9};
   
   virtual_traj = [ virtual_traj; [t,x,y,z] ];
   
   traj_line= fgetl(f_traj);
end

f_obss= fopen('../../records/obss_log0.txt');

if f_obss == -1
    error('File obss_log.txt could not be opened, check name or path.')
end

obss_line= fgetl(f_obss);
obs1= [];
obs2= [];
obs3= [];
addrs = [11259136,11259137,11259138];

while ischar(obss_line)
  % 11259136 406288.0000 3699520.0000 30899.1000 0.0000 61.7333 0.0000 1406173248.8122 640.0800 152.4000 
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
  
  has_obs = [t,x,y,z];
  
  if(address == addrs(1) )
      obs1= [obs1; has_obs];
  elseif(address == addrs(2) )
      obs2= [obs2; has_obs];
  else
      obs3= [obs3; has_obs];     
  end
  
  obss_line= fgetl(f_obss);
  
end

%match time
dis1=[];
dis2=[];
dis3=[];

t_traj= virtual_traj(:,1);

t_obs1= obs1(:,1);
t_obs2= obs2(:,1);
t_obs3= obs3(:,1);

i1= 1;
i2= 1;
i3= 1;

k= 200;

for j=1:length(t_traj)
   
   %obs1
   for i=i1:length(t_obs1)-1
     if t_obs1(i)<= t_traj(j) && t_obs1(i+1)>= t_traj(j)
         dis1(j)= sqrt( (virtual_traj(j,2)-obs1(i,2) )^2 ...
                      + (virtual_traj(j,3)-obs1(i,3) )^2 ...
                      + (virtual_traj(j,4)-obs1(i,4) )^2 ... 
                      );
                  
         if(j==k)         
            disp([virtual_traj(j,2)-obs1(i,2),virtual_traj(j,3)-obs1(i,3),virtual_traj(j,4)-obs1(i,4)]);      
         end 
                  
         i1 = i;
         break;
     end
   end
   
   %obs2
   for i=i2:length(t_obs2)-1
     if t_obs2(i)<= t_traj(j) && t_obs2(i+1)>= t_traj(j)
         dis2(j)= sqrt( (virtual_traj(j,2)-obs2(i,2) )^2 ...
                      + (virtual_traj(j,3)-obs2(i,3) )^2 ...
                      + (virtual_traj(j,4)-obs2(i,4) )^2 ... 
                      );
                  
         if(j==k)         
            disp([virtual_traj(j,2)-obs2(i,2),virtual_traj(j,3)-obs2(i,3),virtual_traj(j,4)-obs2(i,4)]);      
         end       
                  
         i2 = i;
         break;
     end
   end
   
   %obs3
   for i=i3:length(t_obs3)-1
     if t_obs3(i)<= t_traj(j) && t_obs3(i+1)>= t_traj(j)
         dis3(j)= sqrt( (virtual_traj(j,2)-obs3(i,2) )^2 ...
                      + (virtual_traj(j,3)-obs3(i,3) )^2 ...
                      + (virtual_traj(j,4)-obs3(i,4) )^2 ... 
                      );
                  
         if(j==k)         
            disp([virtual_traj(j,2)-obs3(i,2),virtual_traj(j,3)-obs3(i,3),virtual_traj(j,4)-obs3(i,4)]);      
         end        
                  
         i3 = i;
         break;
     end
   end
   
end

%plot
figure;
plot3(virtual_traj(:,2),virtual_traj(:,3),virtual_traj(:,4),'r+' );

% figure;
% subplot(3,1,1);
% plot( t_traj, dis1,'b+-' );
% 
% subplot(3,1,2);
% plot( t_traj, dis2,'b+-' );
% 
% subplot(3,1,3);
% plot( t_traj, dis3,'b+-' );


