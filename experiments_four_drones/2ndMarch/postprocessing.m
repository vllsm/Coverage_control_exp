close all;
clear;
clc;

time='20240122_11';
drone1=load(strcat(time,'2318','.mat'));
drone2=load(strcat(time,'2321','.mat'));
drone3=load(strcat(time,'2324','.mat'));
drone4=load(strcat(time,'2327','.mat'));


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

%figure(2)
open('simulation.fig')
set(gca,'ColorOrderIndex',1)
plot(drone1.drone_one_posxy_x,drone1.drone_one_posxy_y,'linewidth',2)
hold on
plot(drone2.drone_two_posxy_x,drone2.drone_two_posxy_y,'linewidth',2)
hold on
plot(drone3.drone_three_posxy_x,drone3.drone_three_posxy_y,'linewidth',2)
hold on
plot(drone4.drone_four_posxy_x,drone4.drone_four_posxy_y,'linewidth',2)
hold on
set(gca,'ColorOrderIndex',1)
plot(drone1.drone_onedes_x,drone1.drone_onedes_y,'o')
hold on
plot(drone2.drone_twodes_x,drone2.drone_twodes_y,'o')
hold on
plot(drone3.drone_threedes_x,drone3.drone_threedes_y,'o')
hold on
plot(drone4.drone_fourdes_x,drone4.drone_fourdes_y,'o')
set(gca,'ColorOrderIndex',1)
plot(drone1.p_center1_x,drone1.p_center1_y,'p')
hold on
plot(drone2.p_center2_x,drone2.p_center2_y,'p')
hold on
plot(drone3.p_center3_x,drone3.p_center3_y,'p')
hold on
plot(drone4.p_center4_x,drone4.p_center4_y,'p')
axis equal


