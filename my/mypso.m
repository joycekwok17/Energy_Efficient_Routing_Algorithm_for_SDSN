function [Solution]=mypso(S1,S2,D,M,coor_com,coor_pro,E_pro_node,E_com_node,a,b,Y,W,xs,ys)
c1=2;
c2=2;
wmax=0.9;
wmin=0.4;
% w=0.7;
K=1000;
%
CH=zeros(M,D);

for i=1:M              % swarm size:M
    Y1=randperm(a-1);
    Y2=Y1(1:D);
    CH(i,:)=Y(Y2); 
end

X=CH;    
% X% position matrix
V=zeros(M,D);

for i=1:M
    for j=1:D
        V(i,j)=randn;    % velocity matrix
    end
end

Pbest=X;              % initialize pbest and gbest
Gbest=X(1,:); 
for i=2:M
    if myfitness(X(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)< ...
            myfitness(Gbest,D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)
        Gbest=X(i,:);
    end
end

for t=1:K      % iteration
    for i=1:M  
        w=(wmax-wmin)*exp(1/(1+t/K));
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
        if isempty(Dead)~=1  %有死亡pro节点
            [inter,ix,id]=intersect(X(i,:),Dead); %inter为公共元素，ix表示公共元素在X中的位置，id表示公共元素在Dead中位置。
            S00=setdiff(Y,X(i,:));          % S00--尚可选作簇头的pro nodes
            if isempty(inter)~=1            % X(i:)中有死亡节点
                r=randperm(length(S00));      %随机产生矩阵位置
                X(i,ix)=S00(r(1:length(ix))); % length(ix)--出现的死节点个数，
            end
        end
         
        %保证X中元素互不相等
        S00=setdiff(Y,X(i,:));          % S00--尚可选作簇头的pro nodes
        if length(S00)>S1-D
            [c,m]=unique(X(i,:),'first');   % m--重复了的簇头在X（i，:）中第一次出现的序号
            Y0=(1:D);
            m_com=setdiff(Y0,m);        % m_com--X（i，：）中需要更换的簇头序号
            randnum=randperm(length(S00));     %随机产生矩阵位置
            X(i,m_com)=S00(randnum(1:length(m_com))); %可随机从C中提取数据
        end
                
        if myfitness(X(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)<  ...
                myfitness(Pbest(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)
            Pbest(i,:)=X(i,:);          % update pbest
        end
        if myfitness(X(i,:),D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys)< ...
                myfitness(Gbest,D,coor_com,coor_pro,Y,W,E_pro_node,E_com_node,xs,ys);
            Gbest=X(i,:);               % update gbest
        end
    end
end    
Solution=Gbest; 
% Solution% optimal solution
end






    



    
        
        
        

        
            
            



            
            
            



