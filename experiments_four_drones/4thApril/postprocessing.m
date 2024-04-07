close all;
clear;
clc;

% time='20240122_11';
% drone1=load(strcat(time,'2318','.mat'));
% drone2=load(strcat(time,'2321','.mat'));
% drone3=load(strcat(time,'2324','.mat'));
% drone4=load(strcat(time,'2327','.mat'));
time='20240407_11';
drone1=load(strcat(time,'3438','.mat'));
drone2=load(strcat(time,'3439','.mat'));
drone3=load(strcat(time,'3440','.mat'));
drone4=load(strcat(time,'3441','.mat'));

figure(1)
axis([0,size(drone1.mode_one,2),0,2]);
hold on
plot(drone1.mode_one,'linewidth',2)
hold on
plot(drone2.mode_two,'linewidth',2)
hold on
plot(drone3.mode_three,'linewidth',2)
hold on
plot(drone4.mode_four,'linewidth',2)
set(gca,'ColorOrderIndex',1)
%plot(drone1.drone_onedes_x,'linewidth',2)
h=legend('mode one','mode two','mode three','mode four');

figure(2)
%open('simulation1_exp.fig')
set(gca,'ColorOrderIndex',1)
plot(drone1.drone_one_posxy_x(1),drone1.drone_one_posxy_y(1),'.','MarkerSize',20,'Color','#0072BD')
hold on
plot(drone1.drone_one_posxy_x,drone1.drone_one_posxy_y,'linewidth',1,'Color','#0072BD')
hold on
plot(drone1.drone_one_posxy_x(1466),drone1.drone_one_posxy_y(1466),'*','MarkerSize',20,'Color','#0072BD')
hold on
plot(drone2.drone_two_posxy_x(1),drone2.drone_two_posxy_y(1),'.','MarkerSize',20,'Color','#D95319')
hold on
plot(drone2.drone_two_posxy_x,drone2.drone_two_posxy_y,'linewidth',1,'Color','#D95319')
hold on
plot(drone2.drone_two_posxy_x(1466),drone2.drone_two_posxy_y(1466),'*','MarkerSize',20,'Color','#D95319')
hold on
plot(drone3.drone_three_posxy_x(1),drone3.drone_three_posxy_y(1),'.','MarkerSize',20,'Color','#EDB120')
hold on
plot(drone3.drone_three_posxy_x,drone3.drone_three_posxy_y,'linewidth',1,'Color','#EDB120')
hold on
plot(drone3.drone_three_posxy_x(1466),drone3.drone_three_posxy_y(1466),'*','MarkerSize',20,'Color','#EDB120')
hold on
plot(drone4.drone_four_posxy_x(1),drone4.drone_four_posxy_y(1),'.','MarkerSize',20,'Color','#7E2F8E')
hold on
plot(drone4.drone_four_posxy_x,drone4.drone_four_posxy_y,'linewidth',1,'Color','#7E2F8E')
hold on
plot(drone4.drone_four_posxy_x(1466),drone4.drone_four_posxy_y(1466),'.','MarkerSize',20,'Color','#7E2F8E')
hold on
set(gca,'ColorOrderIndex',1)
plot(drone1.drone_onedes_x,drone1.drone_onedes_y,'o','MarkerSize',1.5,'Color','#0072BD')
hold on
plot(drone2.drone_twodes_x,drone2.drone_twodes_y,'o','MarkerSize',1.5,'Color','#D95319')
hold on
plot(drone3.drone_threedes_x,drone3.drone_threedes_y,'o','MarkerSize',1.5,'Color','#EDB120')
hold on
plot(drone4.drone_fourdes_x,drone4.drone_fourdes_y,'o','MarkerSize',1.5,'Color','#7E2F8E')
set(gca,'ColorOrderIndex',1)
plot(drone1.p_center1_x,drone1.p_center1_y,'x','MarkerSize',1.5,'Color','#0072BD')
hold on
plot(drone2.p_center2_x,drone2.p_center2_y,'x','MarkerSize',1.5,'Color','#D95319')
hold on
plot(drone3.p_center3_x,drone3.p_center3_y,'x','MarkerSize',1.5,'Color','#EDB120')
hold on
plot(drone4.p_center4_x,drone4.p_center4_y,'x','MarkerSize',1.5,'Color','#7E2F8E')
axis equal


