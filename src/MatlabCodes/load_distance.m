close all;
clear all;
clc;

A=load('../../records/random3.txt');
dis_v=[];
dis_h=[];

[m,n]=size(A);
% for i=1:m
%   if mod(i,2) == 1
%      dis_v = [dis_v,A(i,:)];
%   else
%      dis_h = [dis_h,A(i,:)];
%   end
% end
for i=1:m
    dis_v = [dis_v,A(i,2)];
    dis_h = [dis_h,A(i,1)];
end

hh= 300;
vv= 50;

figure;
scatter(dis_h,dis_v);
hold on;
grid on;
axis equal;

rectangle('Position',[0,0,300,50]);
xlabel('horizontal(m)');
ylabel('vertical(m)');




