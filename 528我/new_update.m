function [E_pro_node,E_com_node,sum_packet]=new_update(Solution,Y,W,D,coor_pro,E_pro_node,E_com_node,CN,Cluster,dis,Eda,sum_packet)
% update函数运行于pso算法选出最佳簇头集合solution后
% 用于更新剩余能量
Eelec=50e-9;
Efs=10e-12;
Emp=13e-16;
d0=sqrt(Efs/Emp);       % transmission dis threshold
f=4000;                 % bits
xc=0;
yc=0;
sum_packet=sum_packet+D;
%%%%%%%%%%%%% intra-cluster energy consumption%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% update residual energy%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% head nodes

global rest_pro;
rest_pro=setdiff(Y,Solution);
for i=1:D
    E_pro_node(Solution(i))= E_pro_node(Solution(i))-CN(i)*f*(Eelec+Eda); % CH接收成员发来的数据包，能耗
end

% common nodes
for i=1:(length(Y)-D) % rest pro nodes
       %  退化的pro nodes向CH发送数据包的能耗
    if dis(i,Cluster(i))<d0
        E_pro_node(rest_pro(i))=E_pro_node(rest_pro(i))...
            -(Eelec*f+Efs*f*(dis(i,Cluster(i))).^2);
    else
        E_pro_node(rest_pro(i))=E_pro_node(rest_pro(i)) ...
        -(Eelec*f+Emp*f*(dis(i,Cluster(i))).^4);
        % 
    end
end

for i=1:(length(W))             % com nodes向头节点发送数据包的能耗
    dc=dis(length(rest_pro)+i,Cluster(length(rest_pro)+i)); % current distance
    Ec=E_com_node(W(i));                                   % current engergy
    if dc<d0
        E_com_node(W(i))=Ec-(Eelec*f+Efs*f*dc*dc);
    else
        E_com_node(W(i))=Ec-(Eelec*f+Efs*f*dc*dc*dc*dc);
    end
end

[child,judge,parent]=new_routing(Solution,coor_pro,E_pro_node,Y,D,xc,yc,d0);
%%%%%%%%%%%%%%%%%%% inter-cluster energy consumption%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% update residual energy %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:D
    E_pro_node(Solution(i))=E_pro_node(Solution(i))-child(i)*f*(Eelec+Eda); % receiving packets
    for j=1:D
        if judge(i,j)==1
            ju=sqrt((coor_pro(1,Solution(i))-coor_pro(1,Solution(j))).^2+ ...
            (coor_pro(2,Solution(i))-coor_pro(2,Solution(j))).^2);
            if ju<d0
                E_pro_node(Solution(i))=E_pro_node(Solution(i))-...
                    parent(i)*(Eelec*f+Efs*f*ju*ju);                   % transmitting packets
            else 
                E_pro_node(Solution(i))=E_pro_node(Solution(i))-...
                    parent(i)*(Eelec*f+Emp*f*ju*ju*ju*ju) ;
            end
        end
    end
end
%     ju=5;
%     E_pro_node(Solution(i))=E_pro_node(Solution(i))-...   
%                 sum_parent(i)*(Eelec*f+Efs*f*ju*ju);
end





