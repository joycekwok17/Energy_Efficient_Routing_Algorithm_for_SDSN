function [cost]=myfitness(S,D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)
% X--待评估的某组簇头（解）
% fitness function definition
global rest_pro;
rest_pro=setdiff(Y,S);
% sum(E_pro_node(S(1:D)));
% E_new_head=E_pro_node(S(1:D));% residual energy

E_new_com=[E_pro_node(rest_pro(1:(length(Y)-D))),E_com_node(W(1:length(W)))];
Average_E_head=sum(E_pro_node(S(1:D)))/D;
Average_E_com=sum(E_new_com)/(D+length(W));
f1=Average_E_com/Average_E_head;                % f1--ratio of residual energy

dis_head_to_server=zeros(1,D);
dis_com_to_server=zeros(1,length(Y)-D+length(W));
for i=1:D
    dis_head_to_server(i)=sqrt((coor_pro(1,S(i))-xs).^2 ...
        +(coor_pro(2,S(i))-ys).^2);
end
for i=1:(length(Y)-D)
    dis_com_to_server1(i)=sqrt((coor_pro(1,rest_pro(i))-xs).^2 ...
        +(coor_pro(2,rest_pro(i))-ys).^2);
end
for i=1:length(W)
    dis_com_to_server2(i)=sqrt((coor_com(1,W(i))-xs).^2 ...
        +(coor_com(2,W(i)-ys).^2));
end
dis_com_to_server=[dis_com_to_server1,dis_com_to_server2];
AVG_dis_head_to_server=sum(dis_head_to_server)/D;
AVG_dis_com_to_server=sum(dis_com_to_server)/(length(Y)-D+length(W));
f2=AVG_dis_head_to_server/AVG_dis_com_to_server;% f2--ratio of distance from server

A=zeros(1,D);
for i=1:D
    A(i)=(E_pro_node(S(i))-Average_E_head).^2;
end
f3=sqrt(sum(A));                    % f3--energy balance between head nodes

ru1=zeros(1,D);
ru2=zeros(1,length(Y)-D+length(W));
ru1sq=zeros(1,D);
ru2sq=zeros(1,length(Y)-D+length(W));
for i=1:D
    ru1(i)=E_pro_node(S(i))/2;      % 簇头--current energy to original: 2J
    ru1sq(i)=ru1(i).*ru1(i);
end
for i=1:(length(Y)-D)
    ru2_1(i)=E_new_com(i)/2;         % pro nodes中退化为sensor nodes的
    ru2sq_1(i)=ru2_1(i).*ru2_1(i);
end
for i=1:length(W)
    ru2_2(i)=E_new_com(i)/0.5;      % com nodes
    ru2sq_2(i)=ru2_2(i).*ru2_2(i);
end
ru2sq=[ru2sq_1,ru2sq_2];
alpha=0.5;
f4m=alpha*(sum(ru1).^2/(D*sum(ru1sq)))+(1-alpha)* ...
    (sum(ru2).^2/((length(Y)-D+length(W))*sum(ru2sq)));
f4=1/f4m;              % energy balance between head nodes and common nodes
a1=1/4;
a2=1/4;
a3=1/4;
a4=1/4;
cost=a1*f1+a2*f2+a3*f3+a4*f4; % cost
end