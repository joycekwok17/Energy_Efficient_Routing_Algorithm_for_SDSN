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
        %��֤X(i,:)�в����������ڵ�
        Y1=(1:S1);
        Dead=setdiff(Y1,Y);
        if isempty(Dead)~=1  %������pro�ڵ�
            [inter,ix,id]=intersect(X(i,:),Dead); %interΪ����Ԫ�أ�ix��ʾ����Ԫ����X�е�λ�ã�id��ʾ����Ԫ����Dead��λ�á�
            S00=setdiff(Y,X(i,:));          % S00--�п�ѡ����ͷ��pro nodes
            if isempty(inter)~=1            % X(i:)���������ڵ�
                r=randperm(length(S00));      %�����������λ��
                X(i,ix)=S00(r(1:length(ix))); % length(ix)--���ֵ����ڵ������
            end
        end
         
        %��֤X��Ԫ�ػ������
        S00=setdiff(Y,X(i,:));          % S00--�п�ѡ����ͷ��pro nodes
        if length(S00)>S1-D
            [c,m]=unique(X(i,:),'first');   % m--�ظ��˵Ĵ�ͷ��X��i��:���е�һ�γ��ֵ����
            Y0=(1:D);
            m_com=setdiff(Y0,m);        % m_com--X��i����������Ҫ�����Ĵ�ͷ���
            randnum=randperm(length(S00));     %�����������λ��
            X(i,m_com)=S00(randnum(1:length(m_com))); %�������C����ȡ����
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






    



    
        
        
        

        
            
            



            
            
            



