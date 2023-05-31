%% PSO-based
clear;
S1=40; % pro ndoe
S2=100;% com node
D=32; % number of CHs
M=20; %swarm size

xm=100; 
ym=100; % 传感区域范围
xs=0;%控制器位于原点
ys=0;
xc1=-5;%4 controller nodes
yc1=-5;
xc2=-5;
yc2=5;
xc3=5;
yc3=5;
xc4=5;
yc4=-5;
xc=[-5,-5,5,5;-5,5,5,-5];
for i=1:2
    for j=1:S2
        coor_com(i,j)=-xm/2+xm*rand;
    end
end
coor_pro=zeros(2,S1);
for i=1:2
    for j=1:S1
        coor_pro(i,j)=-xm/2+xm*rand;
    end
end
% engergy initialization
E_pro_node(1:S1)=2; 
E_com_node(1:S2)=0.5;
E_con_node(1:4)=10;
% check death head nodes
a=1;
for i=1:S1
    if E_pro_node(i)>0
        Y(a)=i;         % alive pro nodes 序号
        a=a+1  ;        % 存活pro nodes数量
    end
end

b=1;
for i=1:S2
    if E_com_node(i)>0
        W(b)=i;         % alive common nodes序号
        b=b+1;
    end
end

k=1;
sum_packet=0;
while(a>=S1+1)
     if (mod(k,500)==1) %
         [Solution,E_pro_node,E_com_node,CN,Cluster,dis]=mypso(S1,S2,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,b,Y,W,xs,ys);
         [child,judge,sum_parent]=myrouting(Solution,coor_pro,E_pro_node,xc,Y,D);
         [E_pro_node,E_com_node,sum_packet]=myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,...
                xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4,child,judge,sum_parent,CN,Cluster,dis,sum_packet);
     else
         [E_pro_node,E_com_node,sum_packet]=myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,...
               xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4,child,judge,sum_parent,CN,Cluster,dis,sum_packet);% sum_packet是从开始到第k轮为止的sum_packet
     end
     a=1;
     for i=1:S1
         if E_pro_node(i)>0
                Y(a)=i;         % alive pro nodes 序号
                a=a+1;
         end
     end
     current_round_packet(k)=sum_packet;
     k=k+1 % 真正生命长度为k-1
end
% figure(9)
% for i=1:1801-k
%     add1(i)=current_round_packet(k);
% end
% current_round_packet=[current_round_packet,add1]
% plot(1:1800,current_round_packet,'r-o','linewidth',1.5 ,'markersize',2.5);hold on;
% xlabel('时间长度');ylabel('吞吐量/包');
% 
% figure(10)
% add2=zeros(1,k-2);
% for i=1:1802-k
%     add3(i)=1;
% end
% s_dead=[add2,add3];
% plot(1:1800,s_dead,'r-o','linewidth',1.5 ,'markersize',2.5);hold on;
% xlabel('时间长度');ylabel('死亡节点数目')
