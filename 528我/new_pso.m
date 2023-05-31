function [Solution,E_pro_node,E_com_node,CN,Cluster,dis]=new_pso(S1,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,Y,W,xs,ys,xc,yc)
    c1=2;%��������
    c2=2;
    wmax=0.9;           % weight 
    wmin=0.4;
    K=1000;             % iteration 
    CH=zeros(M,D);      % CH matrix�洢��ͷ��ŵ����

    for i=1:M              % swarm size:M
        Y1=randperm(a-1);   %1��a-1�������һ������
        Y2=Y1(1:D);         %ȡǰD������
        CH(i,:)=Y(Y2);      %Y��ǰD���ڵ���Ϊһ��CH
    end

    X=CH;             % =position matrix  
    V=zeros(M,D);   %  ��ʼ�� velocity matrix

    for i=1:M
        for j=1:D
            V(i,j)=randn;    % velocity matrix
        end
    end

    Pbest=X;              % initialize pbest and gbest
    Gbest=X(1,:); 
    
    for i=2:M
        if new_fitness(X(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)< ...
             new_fitness(Gbest,D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)
            Gbest=X(i,:);
        end
    end

    for t=1:K      % iteration������K��
        for i=1:M  
            w=(wmax-wmin)*exp(1/(1+t/K));  % non linear weight 
            w=max(w,wmin);
            w=min(w,wmax);
            V(i,:)=w*V(i,:)+c1*randn*(Pbest(i,:)-X(i,:)) ...
                    +c2*randn*(Gbest(1,:)-X(i,:));           % update velocity
            X(i,:)=mod(round(X(i,:)+V(i,:)),S1);             % update position(cluster heads)
            for j=1:D
                if X(i,j)==0
                    X(i,j)=1;
                end            
            end
            
             %��֤X(i,:)�в����������ڵ�
            Y1=(1:S1);
            Dead=setdiff(Y1,Y);
            if isempty(Dead)~=1                            %��������pro�ڵ�
                [inter,ix,id]=intersect(X(i,:),Dead);       %interΪ����Ԫ�أ�ix��ʾ����Ԫ����X�е�λ�ã�id��ʾ����Ԫ����Dead��λ�á�
                S00=setdiff(Y,X(i,:));                      % S00���п�ѡ����ͷ�ģ����ŵ�pro nodes
                if isempty(inter)~=1                        % X(i:)���������ڵ�
                    r=randperm(length(S00));                % �����������λ��
                    X(i,ix)=S00(r(1:length(ix)));           % length(ix)�����ֵ����ڵ����
                end
            end
         
            %��֤X��Ԫ�ػ������
            S00=setdiff(Y,X(i,:));              % S00���п�ѡ����ͷ��pro nodes
            if length(S00)>S1-D
                [c,m]=unique(X(i,:),'first');   % m���ظ��˵Ĵ�ͷ��X��i��:���е�һ�γ��ֵ����
                Y0=(1:D);
                m_com=setdiff(Y0,m);            % m_com��X��i����������Ҫ�����Ĵ�ͷ���
                randnum=randperm(length(S00));  % �����������λ��
                X(i,m_com)=S00(randnum(1:length(m_com))); %�������C����ȡ���ݣ��滻�ظ��˵�CH
            end
            
            % update pbest    
            if new_fitness(X(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)<  ...
                    new_fitness(Pbest(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)
                Pbest(i,:)=X(i,:);          
            end
            % update gbest
            if new_fitness(X(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)< ...
                    new_fitness(Gbest,D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys);
                Gbest=X(i,:);               
            end
        end
    end    
    Solution=Gbest; % Solution��optimal cluster group

    global rest_pro;
    rest_pro=setdiff(Y,Solution);           % �˻���pro nodes
    global dis;                             % dis matrix��between rest_pro/common nodes,head nodes
    dis=zeros((length(Y)-D+length(W)),D);   % ������󣺷�CH�ڵ��CH֮��ľ���
    for j=1:D
        for i=1:length(rest_pro)                                % �����˻���pro nodes��CH�ľ���
            dis(i,j)=sqrt((coor_pro(1,rest_pro(i))-coor_pro(1,Solution(j))).^2 ...
            +(coor_pro(2,rest_pro(i))-coor_pro(2,Solution(j))).^2);
        end
        for i=1:length(W)                                       % ����com nodes��CH�ľ���
            dis(i+length(rest_pro),j)=sqrt((coor_com(1,W(i))-coor_pro(1,Solution(j))).^2 ...
                +(coor_com(2,W(i))-coor_pro(2,Solution(j))).^2);
        end
    end

% divide common nodes into clusters with shortest distance 
    global Cluster
    Cluster=zeros(1,length(rest_pro)+length(W)); % ͳ�Ʒִ���Ϣ
    for i=1:(length(rest_pro)+length(W))
        d=100000;
        for j=1:D
            if dis(i,j)<d
                Cluster(i)=j; % ���ݾ��룬i�ڵ����ڵ�j����
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
                CN(i)=CN(i)+1; % number of member nodes in each head node ͷ�ڵ�i�еĳ�Ա����
            end
        end
    end
    
    figure(1)
    plot(coor_com(1,:),coor_com(2,:),'or');hold on; % ��ɫԲ�㣺com nodes
    plot(coor_pro(1,:),coor_pro(2,:),'*b');hold on; % ��ɫ���ǣ��˻���pro nodes
    plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on; % head node ����ɫ����
    plot(xc,yc,'hk');hold on;                       % control node����
    plot(xs,ys,'+m');hold on;                       % server��magenta

    % ��Ա�ڵ��CH������,��ͼ��cluster formulation
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
    legend('common sensor nodes','programmable sensor nodes','head nodes','programmable control node','control server');
    title('head node selection and cluster formulation');hold off;
end






    



    
        
        
        

        
            
            



            
            
            



