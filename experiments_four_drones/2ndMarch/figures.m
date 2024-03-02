%% plot to check when is the collision algorithm activated
% note that in the current settings, it seems that only collision between
% agents14 and agents 23 may happen, to verify it, we need to plot all neighbouring collision binary terms. 
    t=0:1:timestep2-1;

% first for agent 1, let's check 12, 13, 14
    figure;
    axis([0,timestep2-1,0,2]);   
    hold on;
    temp12=reshape(collision_his(1,2,1:timestep2),1,timestep2);
    plot(t,temp12,'-.','linewidth',1);
    hold on;
    temp13=reshape(collision_his(1,3,1:timestep2),1,timestep2);
    plot(t,temp13,':','linewidth',1);
    hold on;
    temp14=reshape(collision_his(1,4,1:timestep2),1,timestep2);
    plot(t,temp14,'--','linewidth',1);
    xlabel('\it k','Fontname', 'Times New Roman','FontSize',14);
    ylabel('\it Ag1Collision','Fontname', 'Times New Roman','FontSize',14);
    h=legend('\it col_{12}','\it col_{13}','\it col_{14}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12);

    % first for agent 2, let's check 21, 23, 24
    figure;
    axis([0,timestep2-1,0,2]);   
    hold on;
    temp21=reshape(collision_his(2,1,1:timestep2),1,timestep2);
    plot(t,temp21,'-.','linewidth',1);
    hold on;
    temp23=reshape(collision_his(2,3,1:timestep2),1,timestep2);
    plot(t,temp23,':','linewidth',1);
    hold on;
    temp24=reshape(collision_his(2,4,1:timestep2),1,timestep2);
    plot(t,temp24,'--','linewidth',1);
    xlabel('\it k','Fontname', 'Times New Roman','FontSize',14);
    ylabel('\it Ag2Collision','Fontname', 'Times New Roman','FontSize',14);
    h=legend('\it col_{21}','\it col_{23}','\it col_{24}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12);

    % first for agent 3, let's check 31, 32, 34
    figure;
    axis([0,timestep2-1,0,2]);   
    hold on;
    temp31=reshape(collision_his(3,1,1:timestep2),1,timestep2);
    plot(t,temp31,'-.','linewidth',1);
    hold on;
    temp32=reshape(collision_his(3,2,1:timestep2),1,timestep2);
    plot(t,temp32,':','linewidth',1);
    hold on;
    temp34=reshape(collision_his(3,4,1:timestep2),1,timestep2);
    plot(t,temp34,'--','linewidth',1);
    xlabel('\it k','Fontname', 'Times New Roman','FontSize',14);
    ylabel('\it Ag3Collision','Fontname', 'Times New Roman','FontSize',14);
    h=legend('\it col_{31}','\it col_{32}','\it col_{34}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12);

        % first for agent 4, let's check 11, 42, 43
    figure;
    axis([0,timestep2-1,0,2]);   
    hold on;
    temp41=reshape(collision_his(4,1,1:timestep2),1,timestep2);
    plot(t,temp41,'-.','linewidth',1);
    hold on;
    temp42=reshape(collision_his(4,2,1:timestep2),1,timestep2);
    plot(t,temp42,':','linewidth',1);
    hold on;
    temp43=reshape(collision_his(4,3,1:timestep2),1,timestep2);
    plot(t,temp43,'--','linewidth',1);
    xlabel('\it k','Fontname', 'Times New Roman','FontSize',14);
    ylabel('\it Ag4Collision','Fontname', 'Times New Roman','FontSize',14);
    h=legend('\it col_{41}','\it col_{42}','\it col_{43}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12);