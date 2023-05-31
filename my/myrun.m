S1=40; % pro ndoe
S2=100;% com node
D=10; % number of CHs
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
l=1;

while (a>1)
    if (mod(l,1000)==1)
        Solution=mypso(S1,S2,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,b,Y,W,xs,ys);
    end
    myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4);
    a=1;
    for i=1:S1
        if E_pro_node(i)>0
            Y(a)=i;         % alive pro nodes 序号
            a=a+1  ;        % 
        end
    end

    b=1;
    for i=1:S2
        if E_com_node(i)>0
            W(b)=i;         % alive common nodes序号
            b=b+1;
        end 
    end
    l=l+1
end
