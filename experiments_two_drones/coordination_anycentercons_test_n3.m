function coordination_anycentercons_test_n3()
    close all;
    clear;
    clc;
    global n;
    n=4; %Agents number
    n=3;
    n=2;

    epsilon=0.01;%timestep size
    timestep=1501;%timestep
    timestep2=1501;%draw_step
    global R;%mission_radius
    R=0.1;
    u_max=1;
    
    p=zeros(n,2);%initial positions
    p_dest=zeros(n,2);%initial destination
    u=zeros(n,2);%control input
    dist2R=zeros(n,1);%distance to star
    f=zeros(n,1);%angle convergence
    destcur_angle=zeros(n,1);%destination-current angle
    
    p_start=[-2 2;
        -2 -2;
        4 4;
         2 -2];
%     p_start=[-1 1;
%         -1 -1;
%          1 -1];  
%      p_start=[-1 1;
%         1 -1];
    p_start=[-0.8 0.8;
        0.8 -0.8];
     p=p_start;
   %dest=-p;
    
    k=2;%control gain
    gamma=1;%radius gain
    kx=0.5;%control gain x
    ky=0.5;%control gain y
    %ky=1;
    m=1;%y converagence rate   
    a=[1 1 1 1];%consensus gain
    a=[1 1 1];%consensus gain
    %a=[1 1];%consensus gain
    %a=a*0.5;
    %dest=2*[p_center;p_center;p_center;p_center]-p;
    p_dest=[3 -1;
        1  3;
        -2 -3;
        -3 1];
%     p_dest=[1 -1;
%         1  1;
%         -1 1];
%     p_dest=[1 -0.5;
%         -1  0.5;];
    p_dest=[0.8 -0.5;
        -0.8  0.5;];
     p_center=[1 1];%coordination point
     p_center=0.5*(p_start+p_dest);
     p_centerini=p_center;
    
    
    %trajectories initiallization
    u_his=zeros(n,2,timestep);
    u_norm_his=zeros(n,timestep);
    v_his=zeros(5,timestep);
    p_his=zeros(n,2,timestep);
    f_his=zeros(n,timestep);
    dist2R_his=zeros(n,timestep);
    p_center_his=zeros(n,2,timestep);
    collision_his=zeros(n,n,timestep);
    
    pos_x=zeros(n,timestep);
    pos_y=zeros(n,timestep);
    pos_z_traj=ones(1,timestep);
    

    for s=1:timestep

        
        %cal u

        for i=1:n
%             for j=1:2
%                 u(i,j)=-k*sign(p(i,j)-p_star(1,j))*(norm(p(i,:)-p_star)-R)^m;
%             end
            
            destcur_angle(i)=acos((p_dest(i,:)-p_center(i,:))*(p(i,:)-p_center(i,:))'/norm(p_dest(i,:)-p_center(i,:))/norm(p(i,:)-p_center(i,:)));
            f(i)=destcur_angle(i)/pi;            
            f_his(i,s)=f(i);
            
            dist2R(i)=R^2-norm(p(i,:)-p_center(i,:))^2;                
            dist2R_his(i,s)=dist2R(i);
            
            flag=0;
            p2l=abs((p_dest(i,2)-p_center(i,2))*p(i,1)-(p_dest(i,1)-p_center(i,1))*p(i,2)+(p_dest(i,1)-p_center(i,1))*p_center(i,2)-(p_dest(i,2)-p_center(i,2))*p_center(i,1))/sqrt((p_dest(i,2)-p_center(i,2))^2+(p_dest(i,1)-p_center(i,1))^2);
            %temp=[(p_dest(i,1)-p_center(i,1))/norm(p_dest(i,:)-p_center(i,:)) -(p_dest(i,2)-p_center(i,2))/norm(p_dest(i,:)-p_center(i,:));(p_dest(i,2)-p_center(i,2))/norm(p_dest(i,:)-p_center(i,:)) (p_dest(i,1)-p_center(i,1))/norm(p_dest(i,:)-p_center(i,:))]*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];
            temp=[(p_dest(i,1)-p_center(i,1)) -(p_dest(i,2)-p_center(i,2));(p_dest(i,2)-p_center(i,2)) (p_dest(i,1)-p_center(i,1))]/norm(p_dest(i,:)-p_center(i,:))*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];%ROT*u_xy
            %temp=k*[gamma*dist2R(i) -1;1 gamma*dist2R(i)]*(p(i,:)-p_center(i,:))';
            u(i,:)=temp';
            %他人位置在目标方向之前 且 他人速度与自己速度有相反分量
            for j=1:n
                %if (p(j,:)-p(i,:))*(p_dest(i,:)-p(i,:))' >0
                if (p(i,:)-p(j,:))*(p_dest(i,:)-p(j,:))' <0
                    if u_his(j,:,max(1,s-1))*u_his(i,:,max(1,s-1))'<0
                        flag=1;
                        temp=k*[gamma*dist2R(i) -1;1 gamma*dist2R(i)]*(p(i,:)-p_center(i,:))';%*f(i);
                        u(i,:)=temp';%turn around
                        %                         p_center(i,:)=p_center(i,:)+epsilon*a(i)*(p_center(j,:)-p_center(i,:));%有冲突的互相商议
                        %                         p_center_his(i,:,s)=p_center(i,:);
                        collision_his(i,j,s)=1;
                        break;
                    end
                end
            end        
            
%             if flag==1
%                 temp=k*[gamma*dist2R(i) -1;1 gamma*dist2R(i)]*(p(i,:)-p_center(i,:))';%*f(i);
%                 u(i,:)=temp';%turn around
%             end
            
            
%             if abs(destcur_angle(i)/pi)>40/180
%                 temp=k*[gamma*dist2R(i) -1;1 gamma*dist2R(i)]*(p(i,:)-p_center(i,:))';%*f(i);
%                 u(i,:)=temp';
%             else
%                 p2l=abs((p_dest(i,2)-p_center(i,2))*p(i,1)-(p_dest(i,1)-p_center(i,1))*p(i,2)+(p_dest(i,1)-p_center(i,1))*p_center(i,2)-(p_dest(i,2)-p_center(i,2))*p_center(i,1))/sqrt((p_dest(i,2)-p_center(i,2))^2+(p_dest(i,1)-p_center(i,1))^2);
%                 %temp=[(p_dest(i,1)-p_center(i,1))/norm(p_dest(i,:)-p_center(i,:)) -(p_dest(i,2)-p_center(i,2))/norm(p_dest(i,:)-p_center(i,:));(p_dest(i,2)-p_center(i,2))/norm(p_dest(i,:)-p_center(i,:)) (p_dest(i,1)-p_center(i,1))/norm(p_dest(i,:)-p_center(i,:))]*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];
%                 temp=[(p_dest(i,1)-p_center(i,1)) -(p_dest(i,2)-p_center(i,2));(p_dest(i,2)-p_center(i,2)) (p_dest(i,1)-p_center(i,1))]/norm(p_dest(i,:)-p_center(i,:))*[kx*norm(p_dest(i,:)-p(i,:));ky*p2l^m];%ROT*u_xy
%                 %temp=k*[gamma*dist2R(i) -1;1 gamma*dist2R(i)]*(p(i,:)-p_center(i,:))';
%                 u(i,:)=temp';
%             end
        end
           
        %update status
        for i=1:n
            %v(i)=v(i)+epsilon*u(i);
            u_norm=sqrt(u(i,1)^2+u(i,2)^2);
            if u_norm>u_max
                u(i,1)=u(i,1)/u_norm*u_max;
                u(i,2)=u(i,2)/u_norm*u_max;
            end
            p(i,:)=p(i,:)+epsilon*u(i,:);
            
            u_his(i,:,s)=u(i,:);
            u_norm_his(i,s)=sqrt(u(i,1)^2+u(i,2)^2);
            %v_his(i,s)=v(i);
            p_his(i,:,s)=p(i,:);
            pos_x(i,s)=p(i,1);
            pos_y(i,s)=p(i,2);
            for j=1:n
                p_center(i,:)=p_center(i,:)+epsilon*a(i)*(p_center(j,:)-p_center(i,:));
            end
            p_center_his(i,:,s)=p_center(i,:);

        end


    end
    
    
    
    t=0:1:timestep2-1;
    clf;
    f=1;
    figure(f)
    axis([0,timestep2-1,-0.5,1.5]);
    
    hold on;
    
    plot(t',f_his(1,1:timestep2)','linewidth',2);
    plot(t',f_his(2,1:timestep2)','linewidth',2);
%    plot(t',f_his(3,1:timestep2)','linewidth',2);
%     plot(t',f_his(4,1:timestep2)','linewidth',2);
    %plot(t',q_his(5,1:timestep2)','linewidth',2);
    
    h=legend('\it f_{1}','\it f_{2}','\it f_{3}','\it f_{4}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12)
    box on;
    xlabel('\it k','FontSize',14,'Fontname', 'Times New Roman');
    ylabel('\it f_{i}','FontSize',14,'Fontname', 'Times New Roman');
    get(gca,'xtick');
    set(gca,'xticklabel',get(gca,'xtick'));%科学计数转换
    set(gca,'FontSize',12);
    set(figure(f),'Position',[212,288,700,300]);

    f=f+1;
    figure(f)
    axis([0,timestep2-1,-10,2]);
    
    hold on;
    
    plot(t',dist2R_his(1,1:timestep2)','linewidth',2);
    plot(t',dist2R_his(2,1:timestep2)','linewidth',2);
%    plot(t',dist2R_his(3,1:timestep2)','linewidth',2);
%     plot(t',dist2R_his(4,1:timestep2)','linewidth',2);
    %plot(t',q_his(5,1:timestep2)','linewidth',2);
    
    h=legend('\it dist2R_{1}','\it dist2R_{2}','\it dist2R_{3}','\it dist2R_{4}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12)
    box on;
    xlabel('\it k','FontSize',14,'Fontname', 'Times New Roman');
    ylabel('\it dist2R_{i}','FontSize',14,'Fontname', 'Times New Roman');
    get(gca,'xtick');
    set(gca,'xticklabel',get(gca,'xtick'));%科学计数转换
    set(gca,'FontSize',12);
    set(figure(f),'Position',[212,288,700,300]);
       
    
    f=f+1;
    figure(f)
    axis([0,timestep2-1,0,2]);   
    hold on;
    plot(t,u_norm_his(1,1:timestep2)','linewidth',2);
    plot(t,u_norm_his(2,1:timestep2)','linewidth',2);
%    plot(t,u_norm_his(3,1:timestep2)','linewidth',2);
%     plot(t,u_norm_his(4,1:timestep2)','linewidth',2);
%     plot(t,u_norm_his(5,1:timestep2)','linewidth',2);
    xlabel('\it k','Fontname', 'Times New Roman','FontSize',14);
    ylabel('\it u_{i}','Fontname', 'Times New Roman','FontSize',14);
    h=legend('\it u_{1}','\it u_{2}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12)
    box on;
    get(gca,'xtick');
    set(gca,'xticklabel',get(gca,'xtick'));%科学计数转换
    set(gca,'FontSize',12);
    set(figure(f),'Position',[212,288,700,300]);
    
    f=f+1;
    figure(f)
    axis([0,timestep2-1,0,2]);   
    hold on;
    temp=reshape(collision_his(1,2,1:timestep2),1,timestep2);
    plot(t,temp,'linewidth',2);
    temp=reshape(collision_his(2,1,1:timestep2),1,timestep2);
    plot(t,temp,'o','linewidth',1);
    set(gca,'ColorOrderIndex',1)
    temp=reshape(p_his(1,2,1:timestep2),1,timestep2);
    plot(t,temp,'linewidth',2);
   % plot(t,collision_his(1,2,1:timestep2),'linewidth',2);
    %plot(t,collision_his(1,3,1:timestep2),'linewidth',2);
%    plot(t,u_norm_his(3,1:timestep2)','linewidth',2);
%     plot(t,u_norm_his(4,1:timestep2)','linewidth',2);
%     plot(t,u_norm_his(5,1:timestep2)','linewidth',2);
    xlabel('\it k','Fontname', 'Times New Roman','FontSize',14);
    ylabel('\it col1','Fontname', 'Times New Roman','FontSize',14);
    h=legend('\it col_{12}','\it col_{21}');
    set(h,'Orientation','horizon','Fontname', 'Times New Roman','FontSize',12)
    box on;
    get(gca,'xtick');
    set(gca,'xticklabel',get(gca,'xtick'));%科学计数转换
    set(gca,'FontSize',12);
    set(figure(f),'Position',[212,288,700,300]);   
    
    
        
    
    
    f=f+1;
    figure(f)
    %白色背景
    %axis([-0.5*(max(p_start(:,1))-min(p_start(:,1)))+p_center(1),0.5*(max(p_start(:,1))-min(p_start(:,1)))+p_center(1),-0.5*(max(p_start(:,2))-min(p_start(:,2)))+p_center(2),0.5*(max(p_start(:,2))-min(p_start(:,2)))+p_center(2)]);
    xlabel('x');
    ylabel('y');
    axis equal
    %四周的边框
    box on;
    set(figure(f),'Color',[1,1,1]);
    
    %绘图区域
    t=0:0.02:10;  
    global Nt;
    Nt=size(t,2);
    global x;
    global y;
    x=p_center(1)+R*cos(t(1:Nt));
    y=p_center(2)+R*sin(t(1:Nt));
    global ux;
    global uy;
    ux=cos(t(1:Nt));
    uy=sin(t(1:Nt));
    
    for s=1:(timestep2-1)/100:timestep2
        figure(f)
        cla;
        hold on;
        
        for i=1:n
            plot(p_start(i,1),p_start(i,2),'.','MarkerSize',30);
        end
        set(gca,'ColorOrderIndex',1) 
        for i=1:n
            plot(p_centerini(i,1),p_centerini(i,2),'o','MarkerSize',10);
        end
        set(gca,'ColorOrderIndex',1)
        for i=1:n
            plot(p_center_his(i,1,s),p_center_his(i,2,s),'x','MarkerSize',10);
        end
        set(gca,'ColorOrderIndex',1) 
        for i=1:n
            plot(reshape(p_his(i,1,1:s),1,s),reshape(p_his(i,2,1:s),1,s),'.','MarkerSize',1);%'MarkerSize',10
        end
        %plot(x,y)
        
        frame=getframe(gcf);
        imind=frame2im(frame);
        [imind,cm] = rgb2ind(imind,256);
        if s==1
             imwrite(imind,cm,'cood_anycentercons.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
        else
             imwrite(imind,cm,'cood_anycentercons.gif','gif','WriteMode','append','DelayTime',1e-4);
        end
    end
    
        %save traj.mat
    for i=1:n
        str=strcat( 'traj_' ,  int2str(i) ) ;
        pos_x_traj=pos_x(i,:);
        pos_y_traj=pos_y(i,:);
        save(str,'pos_x_traj', 'pos_y_traj', 'pos_z_traj');
    end
    
end

