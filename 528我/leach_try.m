clear;%清除变量
xm=300;%设置区域为100*100
ym=300;


sink.x=0.5*xm;%sink（汇聚）节点坐标
sink.y=0.5*ym;


n=140 %区域内的节点数目
p=32/140;% 节点成为簇头的概率


%Energy Model (all values in Joules)
Eo=0.5;%节点初始能量
ETX=50*0.000000001;%发射单位报文损耗能量
ERX=50*0.000000001;%接收单位报文损耗能量
Efs=10*0.000000000001;%自由空间能量
Emp=0.0013*0.000000000001;%衰减空间能量
EDA=5*0.000000001;%多路径衰减能量


%Values for Hetereogeneity
m=32/140;%成为高级节点比率


a=3;%参数 %\alpha


rmax=1800;        %最大的轮数
%%%%%%%%%%%%%%%%%%%%%%%%% END OF PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
do=sqrt(Efs/Emp); %计算do 通信半径


%Creation of the random Sensor Network
figure(1);%输出图形
for i=1:1:n %i为矩阵1到n，间距为1
    S(i).xd=rand(1,1)*xm;%1行1列矩阵
    XR(i)=S(i).xd;%随机生成的X轴
    
    S(i).yd=rand(1,1)*ym;
    YR(i)=S(i).yd;%随机生成的Y轴
    
    S(i).G=0;%
    S(i).type='N';%节点类型为普通
    
    temp_rnd0=i;%随机数值
    if (temp_rnd0>=m*n+1) %普通节点的随机选举
        S(i).E=Eo;%设置初始能量为E0
        S(i).ENERGY=0;%普通节点
        plot(S(i).xd,S(i).yd,'b:o');%输出节点，用o表示
        hold on;
    end
    if (temp_rnd0<m*n+1) %高级节点的随机选举
        S(i).E=Eo*(1+a)%设置初始能量为Eo*(1+a)
        S(i).ENERGY=1;%高级节点
        plot(S(i).xd,S(i).yd,'r:+');%输出节点，用+表示
        hold on;
    end
end

%%%%%%%%%%%%只有一个汇聚节点%%%%%%%%%%
S(n+1).xd=sink.x;%汇聚节点X轴坐标
S(n+1).yd=sink.y;%汇聚节点Y轴坐标
plot(S(n+1).xd,S(n+1).yd,'*g'); %画出汇聚节点，用x表示


% figure(1);
countCHs=0;
rcountCHs=0;
cluster=1;
countCHs;
rcountCHs=rcountCHs+countCHs;
flag_first_dead=0;%第一个节点死亡的标志变量

sum_packet=zeros(1,rmax);
e2=0;

for r=0:1:1000      %r为矩阵0到最大，间距为1，循环rmax轮，
    r
    pnrm=( p/ (1+a*m) );%普通节点的选举概率
    padv= ( p*(1+a)/(1+a*m) ); %高级节点的选举概率
    if(mod(r, round(1/pnrm) )==0)%余数为0
        for i=1:1:n%i为矩阵1到n，间距为1
            S(i).G=0;%簇头数目
            S(i).cl=0;
        end
    end
    
    if(mod(r, round(1/padv) )==0)
        for i=1:1:n%i为矩阵1到n，间距为1
            if(S(i).ENERGY==1)
                S(i).G=0;%簇头数目
                S(i).cl=0;
            end
        end
    end
    
    
    hold off;
    
    dead=0;%节点死亡数
    dead_a=0;%高级节点死亡数
    dead_n=0;%普通节点死亡数
    
    packets_TO_BS=0;%传输sink节点报文数
    packets_TO_CH=0;%传输簇头的报文数
    PACKETS_TO_CH(r+1)=0;%每轮传送到簇头的报文数
    PACKETS_TO_BS(r+1)=0;%每轮传送到基站的报文数
    
    figure(4);
    
    
    for i=1:1:n %i为矩阵1到n，间距为1
        if (S(i).E<=0)%检查是否有节点死亡
            plot(S(i).xd,S(i).yd,'red .')%输出节点，用红.表示
            dead=dead+1;%节点死亡数+1
            if(S(i).ENERGY==1)%高级节点
                dead_a=dead_a+1;%高级节点死亡数+1
            end
            if(S(i).ENERGY==0)%普通节点
                dead_n=dead_n+1;%普通节点死亡数+1
            end
            hold on;
        end
        if S(i).E>0%节点能量大于0
            S(i).type='N';
            if (S(i).ENERGY==0)%节点类型为普通
                plot(S(i).xd,S(i).yd,'b:o');    %画出普通节点
            end
            if (S(i).ENERGY==1) %节点类型为高级
                plot(S(i).xd,S(i).yd,'r:+');    %画出高级节点
            end
            hold on;
        end
    end
    plot(S(n+1).xd,S(n+1).yd,'*g');             %画出汇聚节点
%     
    
    STATISTICS(r+1).DEAD=dead;  %r轮后死亡节点数
    DEAD(r+1)=dead;             %r轮后死亡节点数
    DEAD_N(r+1)=dead_n;         %r轮后普通节点死亡数
    DEAD_A(r+1)=dead_a;         %r轮后高级节点死亡数
    
    if (dead==1)%第一个节点死亡
        if(flag_first_dead==0)  %第一个节点死亡周期
            first_dead=r        %%%%%%%%%%第一个节点死亡轮数%%%%%%%%%%%%%%%%
            flag_first_dead=1;  %第一个死亡节点标志
            %%%%%%%%%%%%%%%%%%%%%figure%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             figure(9);
%             for i=1:rmax-first_dead
%                 add(i)=sum_packet(r);
%             end
%             sum_packet=[sum_packet,add];
%             plot(1:rmax,sum_packet,'g-o','linewidth',1.5
%             ,'markersize',2.5);       %吞吐量
%             hold on;
%             
%             figure(2)                       
%             for i=1:(rmax-first_dead)
%                 add5(i)=r2(r);
%             end
%             r2=[r2(1:r),add5];
%             plot(1:rmax,r2,'g-o');hold on;
%             xlabel('round');ylabel('ratio');
%             hold on;
%         end
    end
%     
    
    countCHs=0;     %簇头的个数
    cluster=1;      %簇头的数目
    
    %%%%%%%%%%%%选出簇头%%%%%%%%%%%%%%%%%
    for i=1:1:n     %i为矩阵1到n，间距为1
        if(S(i).E>0)%节点剩余能量大于0
            temp_rand=rand;
            if ( (S(i).G)<=0)%没有簇头
                
                if( ( S(i).ENERGY==0 && ( temp_rand <= ( pnrm / ( 1 - pnrm * mod(r,round(1/pnrm)) )) ) ) )%普通节点的簇头选举
                    
                    countCHs=countCHs+1;                %簇头数+1
                    packets_TO_BS=packets_TO_BS+1;      %传送到基站的计数器+1
                    PACKETS_TO_BS(r+1)=packets_TO_BS;   %每轮传送到基站的计数器=传送到基站的计数器
%                     sum_packet=sum_packet+PACKETS_TO_BS(r+1);%网络存活至今向基站发送的数据包数量之和
                    
                    S(i).type='C';%节点类型为簇头
                    S(i).G=100;
                    C(cluster).xd=S(i).xd;          %簇头X轴坐标
                    C(cluster).yd=S(i).yd;          %簇头Y轴坐标
                    plot(S(i).xd,S(i).yd,'y*');     %输出簇头节点，用黄*表示
                    
                    distance=sqrt( (S(i).xd-(S(n+1).xd) )^2 + (S(i).yd-(S(n+1).yd) )^2 );%计算距离
                    C(cluster).distance=distance;       %距离
                    C(cluster).id=i;                    %簇头的节点编号
                    X(cluster)=S(i).xd;                 %X轴坐标
                    Y(cluster)=S(i).yd;                 %Y轴坐标
                    cluster=cluster+1;                  %簇头总数+1
                    
                    
                    distance;%%%%%%%%%%%%簇头向基站发送数据包，%%%%%%%%%%%%%%%%
                    if (distance>do)                            %簇头向基站发送数据包，距离大于通信半径
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance )); %能量消耗
                        e2=e2+(ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance );
                    end
                    if (distance<=do)                             %距离小于通信半径
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000) + Efs*4000*( distance * distance )); %能量消耗
                        e2=e2+(ETX+EDA)*(4000) + Emp*4000*( distance*distance );
                    end
                end
                
                
                if( ( S(i).ENERGY==1 && ( temp_rand <= ( padv / ( 1 - padv * mod(r,round(1/padv)) )) ) ) )%高级节点簇头选举
                    
                    countCHs=countCHs+1;%簇头数+1
                    packets_TO_BS=packets_TO_BS+1;%传送到基站的计数器+1
                    PACKETS_TO_BS(r+1)=packets_TO_BS;%每轮传送到基站的计数器=传送到基站的计数器
                    
                    S(i).type='C';%节点类型为簇头
                    S(i).G=100;
                    C(cluster).xd=S(i).xd;%簇头X轴坐标
                    C(cluster).yd=S(i).yd;%簇头Y轴坐标
                    plot(S(i).xd,S(i).yd,'y*');%输出节点，用黄*表示
                    
                    distance=sqrt( (S(i).xd-(S(n+1).xd) )^2 + (S(i).yd-(S(n+1).yd) )^2 );%计算距离
                    C(cluster).distance=distance;%距离
                    C(cluster).id=i;%簇头的节点编号
                    X(cluster)=S(i).xd;%X轴坐标
                    Y(cluster)=S(i).yd;%Y轴坐标
                    cluster=cluster+1;%簇头总数+1
                    
                    
                    distance;
                    if (distance>do)%距离大于通信半径
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance )); %簇头向基站发送数据，能量消耗
                        e2=e2+(ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance );
                    end
                    if (distance<=do)%距离小于通信半径
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000)+ Efs*4000*( distance * distance )); %簇头向基站发送数据，能量消耗
                        e2=e2+(ETX+EDA)*(4000)+ Emp*4000*( distance*distance);                        
                    end
                end
                
            end
        end
    end
    STATISTICS(r+1).CLUSTERHEADS=cluster-1; %r轮后簇头数
    CLUSTERHS(r+1)=cluster-1;               %r轮后簇头数
    
    for i=1:1:n
        if ( S(i).type=='N' && S(i).E>0 )   %选举正常节点的相关簇头
            if(cluster-1>=1)                %簇头总数大于2个
                min_dis=sqrt( (S(i).xd-S(n+1).xd)^2 + (S(i).yd-S(n+1).yd)^2 );%两节点间最短距离
                min_dis_cluster=1;%距离最小的簇头数
                for c=1:1:cluster-1
                    temp=min(min_dis,sqrt( (S(i).xd-C(c).xd)^2 + (S(i).yd-C(c).yd)^2 ) );
                    if ( temp<min_dis )
                        min_dis=temp;
                        min_dis_cluster=c;
                    end
                end
                line([S(i).xd,C(min_dis_cluster).xd],[S(i).yd,C(min_dis_cluster).yd])  %%%%%%%%%%%%%连线%%%%%%%%%%%%%%%%%%
                min_dis;
                if (min_dis>do)
                    S(i).E=S(i).E- ( ETX*(4000) + Emp*4000*( min_dis * min_dis * min_dis * min_dis));%普通节点发送数据包，能量消耗
                    e2=e2+ETX*(4000) + Emp*4000*( min_dis * min_dis * min_dis * min_dis);
                end
                if (min_dis<=do)
                    S(i).E=S(i).E- ( ETX*(4000) + Efs*4000*( min_dis * min_dis)); 
                    e2=e2+ETX*(4000) + Emp*4000*( min_dis * min_dis);                    
                end
                
                if(min_dis>0)
                    S(C(min_dis_cluster).id).E = S(C(min_dis_cluster).id).E- ( (ERX + EDA)*4000 ); %对应的簇头接收数据包，融合，能量消耗
                    e2=e2+(ERX + EDA)*4000;                                        
                    PACKETS_TO_CH(r+1)=n-dead-cluster+1;                                           %r轮传送到簇头的报文数
                end
                S(i).min_dis=min_dis;
                S(i).min_dis_cluster=min_dis_cluster;
            end
        end
    end
    hold on;
    e2
    sum_packet(r+1)=(r+1)*4000*32; %r轮后所有有效的数据包数量
    r2(r+1)=sum_packet(r+1)/e2;
    
    countCHs;
    rcountCHs=rcountCHs+countCHs;
    STATISTICS(r+1).ENERGY=0;
    for i=1:1:n%当前节点数
        if S(i).E > 0%如果节点i剩余能量大于0
            STATISTICS(r+1).ENERGY = STATISTICS(r+1).ENERGY +S(i).E;%r轮后节点剩余能量加上节点i的剩余能量
        end
    end
    hold off;
    end

end

