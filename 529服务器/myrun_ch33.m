%% PSO-based
clear;
S1=40; % pro ndoe
S2=100;% com node
D=32; % number of CHs
M=20; %swarm size
f=4000;                 % bits

xm=300; 
ym=300; % ��������Χ
xs=0;%������λ��ԭ��
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
        Y(a)=i;         % alive pro nodes ���
        a=a+1  ;        % ���pro nodes����
    end
end

b=1;
for i=1:S2
    if E_com_node(i)>0
        W(b)=i;         % alive common nodes���
        b=b+1;
    end
end

k=1;
sum_packet=0;
while(a>=S1+1)&&(b>=S2+1)
    if (mod(k,100)==1) %
         [Solution,E_pro_node,E_com_node,CN,Cluster,dis]=mypso(S1,S2,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,b,Y,W,xs,ys);
         [child,judge,sum_parent]=myrouting(Solution,coor_pro,E_pro_node,xc,Y,D);
         [E_pro_node,E_com_node,sum_packet]=myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,...
                xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4,child,judge,sum_parent,CN,Cluster,dis,sum_packet);
     else
         [E_pro_node,E_com_node,sum_packet]=myupdate(Solution,Y,W,D,coor_pro,coor_com,E_pro_node,E_com_node,...
               xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4,child,judge,sum_parent,CN,Cluster,dis,sum_packet);% sum_packet�Ǵӿ�ʼ����k��Ϊֹ��sum_packet
     end
     a=1;
     for i=1:S1
         if E_pro_node(i)>0
                Y(a)=i;         % alive pro nodes ���
                a=a+1;
         end
     end
     current_round_packet(k)=sum_packet*f; %��������
     e1(k)=2*S1+0.5*S2-sum(E_pro_node)-sum(E_com_node);%��������
     r1(k)=current_round_packet(k)/e1(k);%Ч��
     k=k+1 % ������������Ϊk-1
end
for i=1:1801-k
    add1(i)=r1(k-1);
end
r1=[r1,add1];
ii=current_round_packet(k-1)
for i=1:1801-k
    add2(i)=ii;
end
current_round_packet=[current_round_packet,add2];
    
%% LEACH
xm=300;%��������Ϊ100*100
ym=300;


sink.x=0.5*xm;%sink����ۣ��ڵ�����
sink.y=0.5*ym;


n=140 %�����ڵĽڵ���Ŀ
p=32/140;% �ڵ��Ϊ��ͷ�ĸ���


%Energy Model (all values in Joules)
Eo=0.5;%�ڵ��ʼ����
ETX=50*0.000000001;%���䵥λ�����������
ERX=50*0.000000001;%���յ�λ�����������
Efs=10*0.000000000001;%���ɿռ�����
Emp=0.0013*0.000000000001;%˥���ռ�����
EDA=5*0.000000001;%��·��˥������


%Values for Hetereogeneity
m=40/140;%��Ϊ�߼��ڵ����


a=3;%���� %\alpha


rmax=1800;        %��������
%%%%%%%%%%%%%%%%%%%%%%%%% END OF PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
do=sqrt(Efs/Emp); %����do ͨ�Ű뾶


%Creation of the random Sensor Network
% figure(1);%���ͼ��
for i=1:1:n %iΪ����1��n�����Ϊ1
    S(i).xd=rand(1,1)*xm;%1��1�о���
    XR(i)=S(i).xd;%������ɵ�X��
    
    S(i).yd=rand(1,1)*ym;
    YR(i)=S(i).yd;%������ɵ�Y��
    
    S(i).G=0;%
    S(i).type='N';%�ڵ�����Ϊ��ͨ
    
    temp_rnd0=i;%�����ֵ
    if (temp_rnd0>=m*n+1) %��ͨ�ڵ�����ѡ��
        S(i).E=Eo;%���ó�ʼ����ΪE0
        S(i).ENERGY=0;%��ͨ�ڵ�
%         plot(S(i).xd,S(i).yd,'b:o');%����ڵ㣬��o��ʾ
        hold on;
    end
    if (temp_rnd0<m*n+1) %�߼��ڵ�����ѡ��
        S(i).E=Eo*(1+a)%���ó�ʼ����ΪEo*(1+a)
        S(i).ENERGY=1;%�߼��ڵ�
%         plot(S(i).xd,S(i).yd,'r:+');%����ڵ㣬��+��ʾ
        hold on;
    end
end

%%%%%%%%%%%%ֻ��һ����۽ڵ�%%%%%%%%%%
S(n+1).xd=sink.x;%��۽ڵ�X������
S(n+1).yd=sink.y;%��۽ڵ�Y������
% plot(S(n+1).xd,S(n+1).yd,'*g'); %������۽ڵ㣬��x��ʾ


% figure(1);
countCHs=0;
rcountCHs=0;
cluster=1;
countCHs;
rcountCHs=rcountCHs+countCHs;
flag_first_dead=0;%��һ���ڵ������ı�־����

sum_packet=zeros(1,rmax);
e2=0;

for r=0:1:rmax      %rΪ����0����󣬼��Ϊ1��ѭ��rmax�֣�
    r
    pnrm=( p/ (1+a*m) );%��ͨ�ڵ��ѡ�ٸ���
    padv= ( p*(1+a)/(1+a*m) ); %�߼��ڵ��ѡ�ٸ���
    if(mod(r, round(1/pnrm) )==0)%����Ϊ0
        for i=1:1:n%iΪ����1��n�����Ϊ1
            S(i).G=0;%��ͷ��Ŀ
            S(i).cl=0;
        end
    end
    
    if(mod(r, round(1/padv) )==0)
        for i=1:1:n%iΪ����1��n�����Ϊ1
            if(S(i).ENERGY==1)
                S(i).G=0;%��ͷ��Ŀ
                S(i).cl=0;
            end
        end
    end
    
    
%     hold off;
    
    dead=0;%�ڵ�������
    dead_a=0;%�߼��ڵ�������
    dead_n=0;%��ͨ�ڵ�������
    
    packets_TO_BS=0;%����sink�ڵ㱨����
    packets_TO_CH=0;%�����ͷ�ı�����
    PACKETS_TO_CH(r+1)=0;%ÿ�ִ��͵���ͷ�ı�����
    PACKETS_TO_BS(r+1)=0;%ÿ�ִ��͵���վ�ı�����
    
%     figure(4);
    
    
    for i=1:1:n %iΪ����1��n�����Ϊ1
        if (S(i).E<=0)%����Ƿ��нڵ�����
%             plot(S(i).xd,S(i).yd,'red .')%����ڵ㣬�ú�.��ʾ
            dead=dead+1;%�ڵ�������+1
            if(S(i).ENERGY==1)%�߼��ڵ�
                dead_a=dead_a+1;%�߼��ڵ�������+1
            end
            if(S(i).ENERGY==0)%��ͨ�ڵ�
                dead_n=dead_n+1;%��ͨ�ڵ�������+1
            end
            hold on;
        end
        if S(i).E>0%�ڵ���������0
            S(i).type='N';
            if (S(i).ENERGY==0)%�ڵ�����Ϊ��ͨ
%                 plot(S(i).xd,S(i).yd,'b:o');    %������ͨ�ڵ�
            end
            if (S(i).ENERGY==1) %�ڵ�����Ϊ�߼�
%                 plot(S(i).xd,S(i).yd,'r:+');    %�����߼��ڵ�
            end
            hold on;
        end
    end
%     plot(S(n+1).xd,S(n+1).yd,'*g');             %������۽ڵ�
%     
    
    STATISTICS(r+1).DEAD=dead;  %r�ֺ������ڵ���
    DEAD(r+1)=dead;             %r�ֺ������ڵ���
    DEAD_N(r+1)=dead_n;         %r�ֺ���ͨ�ڵ�������
    DEAD_A(r+1)=dead_a;         %r�ֺ�߼��ڵ�������
    
    if (dead==1)%��һ���ڵ�����
        if(flag_first_dead==0)  %��һ���ڵ���������
            first_dead=r        %%%%%%%%%%��һ���ڵ���������%%%%%%%%%%%%%%%%
            flag_first_dead=1;  %��һ�������ڵ��־
            %%%%%%%%%%%%%%%%%%%%%figure%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            figure(9)     
            for i=1:(rmax-first_dead)
                add5(i)=r2(first_dead);
            end
            r2=[r2,add5];
            plot(1:rmax,r1,'r-o');hold on;
            plot(1:rmax,r2,'g-o');hold on;
            xlabel('lifetime');ylabel('Ч�ʣ�������/��������');
            legend('NWPSO-based','Leach')
            hold on;
            
            figure(10);
            for i=1:(rmax-first_dead)
                add(i)=sum_packet(first_dead);
            end
            sum_packet=[sum_packet(1:first_dead),add];
            plot(1:rmax,current_round_packet,'r-o','linewidth',1.5,'markersize',2.5);hold on;
            plot(1:rmax,sum_packet,'g-o','linewidth',1.5,'markersize',2.5);   hold on;    %������
            xlabel('lifetime');ylabel('������');
            legend('NWPSO-based','Leach')
            hold on;
        end
    end
    
    
    countCHs=0;     %��ͷ�ĸ���
    cluster=1;      %��ͷ����Ŀ
    
    %%%%%%%%%%%%ѡ����ͷ%%%%%%%%%%%%%%%%%
    for i=1:1:n     %iΪ����1��n�����Ϊ1
        if(S(i).E>0)%�ڵ�ʣ����������0
            temp_rand=rand;
            if ( (S(i).G)<=0)%û�д�ͷ
                
                if( ( S(i).ENERGY==0 && ( temp_rand <= ( pnrm / ( 1 - pnrm * mod(r,round(1/pnrm)) )) ) ) )%��ͨ�ڵ�Ĵ�ͷѡ��
                    
                    countCHs=countCHs+1;                %��ͷ��+1
                    packets_TO_BS=packets_TO_BS+1;      %���͵���վ�ļ�����+1
                    PACKETS_TO_BS(r+1)=packets_TO_BS;   %ÿ�ִ��͵���վ�ļ�����=���͵���վ�ļ�����
%                     sum_packet=sum_packet+PACKETS_TO_BS(r+1);%�������������վ���͵����ݰ�����֮��
                    
                    S(i).type='C';%�ڵ�����Ϊ��ͷ
                    S(i).G=100;
                    C(cluster).xd=S(i).xd;          %��ͷX������
                    C(cluster).yd=S(i).yd;          %��ͷY������
%                     plot(S(i).xd,S(i).yd,'y*');     %�����ͷ�ڵ㣬�û�*��ʾ
                    
                    distance=sqrt( (S(i).xd-(S(n+1).xd) )^2 + (S(i).yd-(S(n+1).yd) )^2 );%�������
                    C(cluster).distance=distance;       %����
                    C(cluster).id=i;                    %��ͷ�Ľڵ���
                    X(cluster)=S(i).xd;                 %X������
                    Y(cluster)=S(i).yd;                 %Y������
                    cluster=cluster+1;                  %��ͷ����+1
                    
                    
                    distance;%%%%%%%%%%%%��ͷ���վ�������ݰ���%%%%%%%%%%%%%%%%
                    if (distance>do)                            %��ͷ���վ�������ݰ����������ͨ�Ű뾶
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance )); %��������
                        e2=e2+(ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance );
                    end
                    if (distance<=do)                             %����С��ͨ�Ű뾶
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000) + Efs*4000*( distance * distance )); %��������
                        e2=e2+(ETX+EDA)*(4000) + Emp*4000*( distance*distance );
                    end
                end
                
                
                if( ( S(i).ENERGY==1 && ( temp_rand <= ( padv / ( 1 - padv * mod(r,round(1/padv)) )) ) ) )%�߼��ڵ��ͷѡ��
                    
                    countCHs=countCHs+1;%��ͷ��+1
                    packets_TO_BS=packets_TO_BS+1;%���͵���վ�ļ�����+1
                    PACKETS_TO_BS(r+1)=packets_TO_BS;%ÿ�ִ��͵���վ�ļ�����=���͵���վ�ļ�����
                    
                    S(i).type='C';%�ڵ�����Ϊ��ͷ
                    S(i).G=100;
                    C(cluster).xd=S(i).xd;%��ͷX������
                    C(cluster).yd=S(i).yd;%��ͷY������
%                     plot(S(i).xd,S(i).yd,'y*');%����ڵ㣬�û�*��ʾ
                    
                    distance=sqrt( (S(i).xd-(S(n+1).xd) )^2 + (S(i).yd-(S(n+1).yd) )^2 );%�������
                    C(cluster).distance=distance;%����
                    C(cluster).id=i;%��ͷ�Ľڵ���
                    X(cluster)=S(i).xd;%X������
                    Y(cluster)=S(i).yd;%Y������
                    cluster=cluster+1;%��ͷ����+1
                    
                    
                    distance;
                    if (distance>do)%�������ͨ�Ű뾶
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance )); %��ͷ���վ�������ݣ���������
                        e2=e2+(ETX+EDA)*(4000) + Emp*4000*( distance*distance*distance*distance );
                    end
                    if (distance<=do)%����С��ͨ�Ű뾶
                        S(i).E=S(i).E- ( (ETX+EDA)*(4000)+ Efs*4000*( distance * distance )); %��ͷ���վ�������ݣ���������
                        e2=e2+(ETX+EDA)*(4000)+ Emp*4000*( distance*distance);                        
                    end
                end
                
            end
        end
    end
    STATISTICS(r+1).CLUSTERHEADS=cluster-1; %r�ֺ��ͷ��
    CLUSTERHS(r+1)=cluster-1;               %r�ֺ��ͷ��
    
    for i=1:1:n
        if ( S(i).type=='N' && S(i).E>0 )   %ѡ�������ڵ����ش�ͷ
            if(cluster-1>=1)                %��ͷ��������2��
                min_dis=sqrt( (S(i).xd-S(n+1).xd)^2 + (S(i).yd-S(n+1).yd)^2 );%���ڵ����̾���
                min_dis_cluster=1;%������С�Ĵ�ͷ��
                for c=1:1:cluster-1
                    temp=min(min_dis,sqrt( (S(i).xd-C(c).xd)^2 + (S(i).yd-C(c).yd)^2 ) );
                    if ( temp<min_dis )
                        min_dis=temp;
                        min_dis_cluster=c;
                    end
                end
%                 line([S(i).xd,C(min_dis_cluster).xd],[S(i).yd,C(min_dis_cluster).yd])  %%%%%%%%%%%%%����%%%%%%%%%%%%%%%%%%
                min_dis;
                if (min_dis>do)
                    S(i).E=S(i).E- ( ETX*(4000) + Emp*4000*( min_dis * min_dis * min_dis * min_dis));%��ͨ�ڵ㷢�����ݰ�����������
                    e2=e2+ETX*(4000) + Emp*4000*( min_dis * min_dis * min_dis * min_dis);
                end
                if (min_dis<=do)
                    S(i).E=S(i).E- ( ETX*(4000) + Efs*4000*( min_dis * min_dis)); 
                    e2=e2+ETX*(4000) + Emp*4000*( min_dis * min_dis);                    
                end
                
                if(min_dis>0)
                    S(C(min_dis_cluster).id).E = S(C(min_dis_cluster).id).E- ( (ERX + EDA)*4000 ); %��Ӧ�Ĵ�ͷ�������ݰ����ںϣ���������
                    e2=e2+(ERX + EDA)*4000;                                        
                    PACKETS_TO_CH(r+1)=n-dead-cluster+1;                                           %r�ִ��͵���ͷ�ı�����
                end
                S(i).min_dis=min_dis;
                S(i).min_dis_cluster=min_dis_cluster;
            end
        end
    end
    hold on;
    e2
    sum_packet(r+1)=(r+1)*4000*32; %r�ֺ�������Ч�����ݰ�����
    r2(r+1)=sum_packet(r+1)/e2;
    
    countCHs;
    rcountCHs=rcountCHs+countCHs;
    STATISTICS(r+1).ENERGY=0;
    for i=1:1:n%��ǰ�ڵ���
        if S(i).E > 0%����ڵ�iʣ����������0
            STATISTICS(r+1).ENERGY = STATISTICS(r+1).ENERGY +S(i).E;%r�ֺ�ڵ�ʣ���������Ͻڵ�i��ʣ������
        end
    end
    hold off;
 end



