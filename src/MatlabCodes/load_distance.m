close all;
clear all;
clc;

A=load('../../records/divide_record.txt');
dis_v=[];
dis_h=[];

[m,n]=size(A);
for i=1:m
  if mod(i,2) == 1
     dis_v = [dis_v,A(i,:)];
  else
     dis_h = [dis_h,A(i,:)];
  end
end

vv= 300;
hh= 50;

figure;
scatter(dis_v,dis_h);
hold on;

rectangle('Position',[0,0,300,50]);




