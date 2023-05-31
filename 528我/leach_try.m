clear;%�������
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
m=32/140;%��Ϊ�߼��ڵ����


a=3;%���� %\alpha


rmax=1800;        %��������
%%%%%%%%%%%%%%%%%%%%%%%%% END OF PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
do=sqrt(Efs/Emp); %����do ͨ�Ű뾶


%Creation of the random Sensor Network
figure(1);%���ͼ��
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
        plot(S(i).xd,S(i).yd,'b:o');%����ڵ㣬��o��ʾ
        hold on;
    end
    if (temp_rnd0<m*n+1) %�߼��ڵ�����ѡ��
        S(i).E=Eo*(1+a)%���ó�ʼ����ΪEo*(1+a)
        S(i).ENERGY=1;%�߼��ڵ�
        plot(S(i).xd,S(i).yd,'r:+');%����ڵ㣬��+��ʾ
        hold on;
    end
end

%%%%%%%%%%%%ֻ��һ����۽ڵ�%%%%%%%%%%
S(n+1).xd=sink.x;%��۽ڵ�X������
S(n+1).yd=sink.y;%��۽ڵ�Y������
plot(S(n+1).xd,S(n+1).yd,'*g'); %������۽ڵ㣬��x��ʾ


% figure(1);
countCHs=0;
rcountCHs=0;
cluster=1;
countCHs;
rcountCHs=rcountCHs+countCHs;
flag_first_dead=0;%��һ���ڵ������ı�־����

sum_packet=zeros(1,rmax);
e2=0;

for r=0:1:1000      %rΪ����0����󣬼��Ϊ1��ѭ��rmax�֣�
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
    
    
    hold off;
    
    dead=0;%�ڵ�������
    dead_a=0;%�߼��ڵ�������
    dead_n=0;%��ͨ�ڵ�������
    
    packets_TO_BS=0;%����sink�ڵ㱨����
    packets_TO_CH=0;%�����ͷ�ı�����
    PACKETS_TO_CH(r+1)=0;%ÿ�ִ��͵���ͷ�ı�����
    PACKETS_TO_BS(r+1)=0;%ÿ�ִ��͵���վ�ı�����
    
    figure(4);
    
    
    for i=1:1:n %iΪ����1��n�����Ϊ1
        if (S(i).E<=0)%����Ƿ��нڵ�����
            plot(S(i).xd,S(i).yd,'red .')%����ڵ㣬�ú�.��ʾ
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
                plot(S(i).xd,S(i).yd,'b:o');    %������ͨ�ڵ�
            end
            if (S(i).ENERGY==1) %�ڵ�����Ϊ�߼�
                plot(S(i).xd,S(i).yd,'r:+');    %�����߼��ڵ�
            end
            hold on;
        end
    end
    plot(S(n+1).xd,S(n+1).yd,'*g');             %������۽ڵ�
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
%             figure(9);
%             for i=1:rmax-first_dead
%                 add(i)=sum_packet(r);
%             end
%             sum_packet=[sum_packet,add];
%             plot(1:rmax,sum_packet,'g-o','linewidth',1.5
%             ,'markersize',2.5);       %������
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
                    plot(S(i).xd,S(i).yd,'y*');     %�����ͷ�ڵ㣬�û�*��ʾ
                    
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
                    plot(S(i).xd,S(i).yd,'y*');%����ڵ㣬�û�*��ʾ
                    
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
                line([S(i).xd,C(min_dis_cluster).xd],[S(i).yd,C(min_dis_cluster).yd])  %%%%%%%%%%%%%����%%%%%%%%%%%%%%%%%%
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

end

