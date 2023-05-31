function [Solution,E_pro_node,E_com_node,CN,Cluster,dis]=new_pso(S1,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,Y,W,xs,ys,xc,yc)
    c1=2;%加速因子
    c2=2;
    wmax=0.9;           % weight 
    wmin=0.4;
    K=1000;             % iteration 
    CH=zeros(M,D);      % CH matrix存储簇头序号的组合

    for i=1:M              % swarm size:M
        Y1=randperm(a-1);   %1到a-1的随机的一个排列
        Y2=Y1(1:D);         %取前D个数字
        CH(i,:)=Y(Y2);      %Y中前D个节点作为一组CH
    end

    X=CH;             % =position matrix  
    V=zeros(M,D);   %  初始化 velocity matrix

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

    for t=1:K      % iteration，迭代K次
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
            
             %保证X(i,:)中不包括死亡节点
            Y1=(1:S1);
            Dead=setdiff(Y1,Y);
            if isempty(Dead)~=1                            %有死亡的pro节点
                [inter,ix,id]=intersect(X(i,:),Dead);       %inter为公共元素，ix表示公共元素在X中的位置，id表示公共元素在Dead中位置。
                S00=setdiff(Y,X(i,:));                      % S00：尚可选作簇头的，活着的pro nodes
                if isempty(inter)~=1                        % X(i:)中有死亡节点
                    r=randperm(length(S00));                % 随机产生矩阵位置
                    X(i,ix)=S00(r(1:length(ix)));           % length(ix)：出现的死节点个数
                end
            end
         
            %保证X中元素互不相等
            S00=setdiff(Y,X(i,:));              % S00：尚可选作簇头的pro nodes
            if length(S00)>S1-D
                [c,m]=unique(X(i,:),'first');   % m：重复了的簇头在X（i，:）中第一次出现的序号
                Y0=(1:D);
                m_com=setdiff(Y0,m);            % m_com：X（i，：）中需要更换的簇头序号
                randnum=randperm(length(S00));  % 随机产生矩阵位置
                X(i,m_com)=S00(randnum(1:length(m_com))); %可随机从C中提取数据，替换重复了的CH
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
    Solution=Gbest; % Solution：optimal cluster group

    global rest_pro;
    rest_pro=setdiff(Y,Solution);           % 退化的pro nodes
    global dis;                             % dis matrix：between rest_pro/common nodes,head nodes
    dis=zeros((length(Y)-D+length(W)),D);   % 距离矩阵：非CH节点和CH之间的距离
    for j=1:D
        for i=1:length(rest_pro)                                % 计算退化的pro nodes与CH的距离
            dis(i,j)=sqrt((coor_pro(1,rest_pro(i))-coor_pro(1,Solution(j))).^2 ...
            +(coor_pro(2,rest_pro(i))-coor_pro(2,Solution(j))).^2);
        end
        for i=1:length(W)                                       % 计算com nodes与CH的距离
            dis(i+length(rest_pro),j)=sqrt((coor_com(1,W(i))-coor_pro(1,Solution(j))).^2 ...
                +(coor_com(2,W(i))-coor_pro(2,Solution(j))).^2);
        end
    end

% divide common nodes into clusters with shortest distance 
    global Cluster
    Cluster=zeros(1,length(rest_pro)+length(W)); % 统计分簇信息
    for i=1:(length(rest_pro)+length(W))
        d=100000;
        for j=1:D
            if dis(i,j)<d
                Cluster(i)=j; % 根据距离，i节点属于第j个簇
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
                CN(i)=CN(i)+1; % number of member nodes in each head node 头节点i中的成员个数
            end
        end
    end
    
    figure(1)
    plot(coor_com(1,:),coor_com(2,:),'or');hold on; % 红色圆点：com nodes
    plot(coor_pro(1,:),coor_pro(2,:),'*b');hold on; % 蓝色星星：退化的pro nodes
    plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on; % head node ：绿色星星
    plot(xc,yc,'hk');hold on;                       % control node：黑
    plot(xs,ys,'+m');hold on;                       % server：magenta

    % 成员节点和CH间连线,画图，cluster formulation
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






    



    
        
        
        

        
            
            



            
            
            



