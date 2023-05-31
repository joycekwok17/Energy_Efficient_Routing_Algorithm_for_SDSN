function myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4)
% update函数运行于pso算法选出最佳簇头集合solution后
% 用于更新分簇情况，距离，剩余能量
% dis matrix--between rest_pro/common nodes,head nodes
Eelec=50e-9;
Efs=10e-12;
Emp=13e-16;
d0=sqrt(Efs/Emp);% transmission dis threshold
f=4000;%bits
global rest_pro;
rest_pro=setdiff(Y,Solution);
global dis;
dis=zeros((length(Y)-D+length(W)),D); 
for j=1:D
    for i=1:length(rest_pro)                            % rest pro nodes
        dis(i,j)=sqrt((coor_pro(1,rest_pro(i))-coor_pro(1,Solution(j))).^2 ...
        +(coor_pro(2,rest_pro(i))-coor_pro(2,Solution(j))).^2);
    end
    for i=1:length(W)                                     % com nodes
        dis(i+length(rest_pro),j)=sqrt((coor_com(1,W(i))-coor_pro(1,Solution(j))).^2 ...
            +(coor_com(2,W(i))-coor_pro(2,Solution(j))).^2);
    end
end

% divide common nodes into clusters with shortest distance 
Cluster=zeros(1,length(rest_pro)+length(W));
for i=1:(length(rest_pro)+length(W))
    d=100000;
    for j=1:D
        if dis(i,j)<d
            Cluster(i)=j;
            d=dis(i,j);
        end
    end
end

% calculate number of member nodes in each head node
global CN;
CN=zeros(1,D);
for i=1:D   
    for j=1:(length(rest_pro)+length(W))
        if Cluster(j)== i
            CN(i)=CN(i)+1;% number of member nodes in each head node
        end
    end
end

% intra-cluster energy consumption-- update residual energy 
% head nodes
for i=1:D
    E_pro_node(Solution(i))= E_pro_node(Solution(i))- CN(i)*f*Eelec;     
end
% common nodes
for i=1:(length(Y)-D) % rest pro nodes
    if dis(i,Cluster(i))<d0
        E_pro_node(rest_pro(i))=E_pro_node(rest_pro(i))...
            -(Eelec*f+Efs*f*(dis(i,Cluster(i))).^2);
    else
        E_pro_node(rest_pro(i))=E_pro_node(rest_pro(i)) ...
        -(Eelec*f+Emp*f*(dis(i,Cluster(i))).^4);
    end
end
% for i=(length(rest_pro)+1):(length(Y)+length(W))
for i=1:(length(W))
    dc=dis(length(rest_pro)+i,Cluster(length(rest_pro)+i)); % current distance
    Ec=E_com_node(W(i));                                   % current engergy
    if dc<d0
        E_com_node(W(i))=Ec-(Eelec*f+Efs*f*dc*dc);
    else
        E_com_node(W(i))=Ec-(Eelec*f+Efs*f*dc*dc*dc*dc);
    end
end

figure(2)
plot(coor_com(1,:),coor_com(2,:),'or');hold on;
plot(coor_pro(1,:),coor_pro(2,:),'*b');hold on;
plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on; % head node selection
plot(xc1,yc1,'hk',xc2,yc2,'hk',xc3,yc3,'hk',xc4,yc4,'hk');hold on;

xs=0;%控制器位于原点
ys=0;
plot(xs,ys,'+m');hold on;
%连线,画图cluster formulation
for i=1:D    
        for j=1:length(rest_pro)
            if Cluster(j)==i 
                line([coor_pro(1,rest_pro(j)),coor_pro(1,Solution(i))],...
                    [coor_pro(2,rest_pro(j)),coor_pro(2,Solution(i))]);   
            end
        end
        for j=(length(rest_pro)+1):(length(rest_pro)+length(W))
            if Cluster(j)==i
                line([coor_com(1,W(j-length(rest_pro))),coor_pro(1,Solution(i))],...
                    [coor_com(2,W(j-length(rest_pro))),coor_pro(2,Solution(i))])
            end
        end
end
% legend('common sensor nodes','programmable sensor nodes','head nodes','programmable controller nodes','control server');
% title('head node selection and cluster formulation');hold off;
xc=[-5,-5,5,5;-5,5,5,-5];
myrouting(Solution,coor_pro,E_pro_node,xc,Y,D);
% inter-cluster energy consumption-- update residual energy 





