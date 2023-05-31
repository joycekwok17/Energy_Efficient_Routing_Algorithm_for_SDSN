%--------适应度函数--------------
function [result1,result2,result3,E_com_t,E_pro_r] = fitness(X,D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor)
% S1:pro sensor node数量；
% S2：com sensor node数量；
% D：簇头个数；
% N：sensor node个数
% X:作为某组簇头的节点的序号
% Y为存活pro sensor node序号


%---------初始化参数设置-----------------------------
E_sensor_t=zeros(1,N);   %单个sensor node发送数据所需的能量
E_head_r=zeros(1,D);     %单个head node接收数据所需的能量

E_pro_r=zeros(1,S1);
E_com_t=zeros(1,S2);

E1=0.5;     %common sensor nodes初始化能量,单位J
E2=2;     %programmable sensor nodes初始化能量

Eelec=50*10^(-9);
Efs=10*10^(-12);
Emp=0.0013*10^(-12);
d0=sqrt(Efs/Emp);        %发送数据能量消耗临界距离

f=4000;                  %4000bit

ru=zeros(1,N);           %节点的能量负载情况
ru2=zeros(1,N);

%------确定非head nodes set, including common sensor nodes and unchosen
%programmable sensor nodes---------------------------
%C=setdiff(Y,X);
C=Compset(Y,X);

%------确定每个sensor nodes所对应的head node-------------------
%------能量限制------------------------------------------------
cluster_head=zeros(1,S2);       %N维数组，用来存放每个sensor nodes所对应的head nodes
d=zeros(D,N);                %D*N矩阵,用来存放head nodes和sensor nodes的距离
% ru_sum=0;
% ru2_sum=0;

%----------------确定common sensor nodes所对应的head nodes
a1=0;
 for k=1:S2              
     dis=1000;
     if E_com_sensor(k)>0             %判断节点是否死亡
         a1=a1+1;
        for i=1:D
            d(i,k)=sqrt((coor_pro(1,X(i))-coor_common(1,k)).^2+(coor_pro(2,X(i))-coor_common(2,k)).^2);
            if d(i,k)<dis
                dis=d(i,k);
                cluster_head(k)=i;
            end
        end
        if dis<d0
            E_sensor_t(k)=Eelec*f+Efs*f*dis*dis;
        else
            E_sensor_t(k)=Eelec*f+Emp*f*dis*dis*dis*dis;
        end
        E_com_t(k)=E_sensor_t(k);
        ru(k)=(E_com_sensor(k)-E_sensor_t(k))/E1;

        ru2(k)=ru(k)*ru(k);
    	ru_sum=ru_sum+ru(k);
        ru2_sum=ru2_sum+ru2(k);
     else
        cluster_head(k)=0;
        E_com_t(k)=0;
     end         
 end
 jain1=ru_sum.^2/(a1*ru2_sum);          %common sensor nodes jain公平指数

%----------确定 unchosen programmable sensor nodes所对应的head nodes
a2=0;
qu_sum=0;
qu2_sum=0;
 for k=1:length(C)      
     dis=1000;
     if E_pro_sensor(k)>0
         a2=a2+1;
        for i=1:D
            d2(i,k)=sqrt((coor_pro(1,X(i))-coor_pro(1,C(k))).^2+(coor_pro(2,X(i))-coor_pro(2,C(k))).^2);
            if d2(i,k)<dis
                dis=d2(i,k);
                cluster_head(k+S2)=i;
            end
        end
        if dis<d0
            E_sensor_t(k+S2)=Eelec*f+Efs*f*dis*dis;
        else
            E_sensor_t(k+S2)=Eelec*f+Emp*f*dis*dis*dis*dis;
        end
        E_pro_r(C(k))=E_sensor_t(k+S2);
        qu(k)=(E_pro_sensor(C(k))-E_sensor_t(k+S2))/E2;
        qu2(k)=qu(k)*qu(k);
        qu_sum=qu_sum+qu(k);
        qu2_sum=qu2_sum+qu2(k);
     else
        cluster_head(k+S2)=0; 
        E_pro_r(C(k))=0;
     end
 end

 pu=zeros(1,D);
 pu2=zeros(1,D);
 pu_sum=0;
 pu2_sum=0;
%------确定每个head node所属的common nodes---------------------
cluster=zeros(D,N);       %D*N矩阵
for i=1:D
    a=1;
    for k=1:S2+length(C)
        if cluster_head(k)==i
            cluster(i,a)=k;
            E_head_r(i)=E_head_r(i)+Eelec*f;
            a=a+1;
        end
    end
    E_pro_r(X(i))=E_head_r(i);
    pu(i)=(E_pro_sensor(X(i))-E_head_r(i))/E2;
    pu2(i)=pu(i)*pu(i);
    pu_sum=pu_sum+pu(i);
    pu2_sum=pu2_sum+pu2(i);
end

jain2=(qu_sum+pu_sum).^2/((a2+D)*(qu_sum+pu2_sum));          %jain公平指数

result1=jain1+jain2;
result2=cluster;
result3=C;

