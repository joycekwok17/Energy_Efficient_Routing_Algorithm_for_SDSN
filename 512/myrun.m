clear;
life_sum=zeros(1,9);
life_avr=zeros(1,9);
for D=28:36     % 实际是k的循环
    k=(D-27);
    life(k)=1;
S1=40; % pro ndoe
S2=100;% com node
% D=35; % number of CHs
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

for j=1:50                  % 对每个k，仿真50次
    while(a>=S1+1)          % 仿真一次的生命周期
        if (mod(life(k),100)==1)  % 簇头更新的周期
            [Solution,E_pro_node,E_com_node,CN,Cluster,dis]=mypso(S1,S2,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,b,Y,W,xs,ys);
            [child,judge,sum_parent]=myrouting(Solution,coor_pro,E_pro_node,xc,Y,D);
            [E_pro_node,E_com_node]=myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,...
                    xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4,child,judge,sum_parent,CN,Cluster,dis);
        else
            [E_pro_node,E_com_node]=myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,...
                    xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4,child,judge,sum_parent,CN,Cluster,dis);
        end
        a=1;
        for i=1:S1
            if E_pro_node(i)>0
                Y(a)=i;         % alive pro nodes 序号
                a=a+1;
            end
        end 
        life(k)=life(k)+1  ;    % 
%         a=a+1;
    end
    life_sum(k)=life_sum(k)+life(k);
%     life(k)

end
    life_avr(k)=life_sum(k)/50
end
figure(3)
plot(28:36,life_avr,'r-o','linewidth',1.5 ,'markersize',2.5);
title('不同的簇头数量对应的网络寿命')
xlabel('簇头数量');
ylabel('网络寿命/通信轮次')

