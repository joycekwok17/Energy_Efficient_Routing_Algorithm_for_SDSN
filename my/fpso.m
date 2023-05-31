function [E_pro,E_com,Solution]=fpso(D,M,N,S1,S2,E_pro_sensor,E_com_sensor,coor_common,coor_pro,coor_con)
% S1:pro sensor node数量；
% S2：com sensor node数量；
% D：簇头个数；
% M：粒子个数；
% N：sensor node个数
%------给定初始化条件----------------------------------------------
c1=2;             %学习因子1
c2=2;             %学习因子2
w=0.7;            %惯性权重
MaxDT=1000;       %最大迭代次数
% eps=10^(-6);      %设置精度(在已知最小值时候用)

aaa=1;
for i=1:S1
    if E_pro_sensor(i)>0
        Y(aaa)=i;           % Y为存活pro sensor node序号
        aaa=aaa+1;
    end
end

Y1=[1:S1];

x=zeros(M,D);     %%
v=zeros(M,D);

%------初始化种群的个体(可以在这里限定位置和速度的范围)------------
for i=1:M
    PA=randperm(length(Y));
    x(i,:)=Y(PA(1:D)); 
    for j=1:D
        v(i,j)=randn;
    end
end
            
%------先计算各个粒子的适应度，并初始化p(i)和gbest--------------------
for i=1:M
    p(i)=fitness(x(i,:),D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor);   
    y(i,:)=x(i,:);
end

gbest=x(1,:);             %gbest为全局最优，初始化gbest
for i=2:M
    if fitness(x(i,:),D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor) > fitness(gbest,D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor)
        gbest=x(i,:);
    end
end

%------进入主要循环，按照公式依次迭代，直到满足精度要求------------
for t=1:MaxDT
    for i=1:M
        v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(gbest-x(i,:));
        x(i,:)=mod(round(x(i,:)+v(i,:)),S1);
        for j=1:D
            if x(i,j)==0;
               x(i,j)=1;
            end            
        end
        
        %保证x中不包括死亡节点
        DEAD=Compset(Y1,Y);       %死亡节点序号
        if isempty(DEAD)~=1       %DEAD不为空
            [inter,ix,id]=intersect(x(i,:),DEAD); %inter为公共元素，ix表示公共元素在x中的位置，id表示公共元素在DEAD中位置。
            C=Compset(Y,x(i,:)); 
            if isempty(inter)~=1
                a=randperm(length(C)); %随机产生矩阵位置
                x(i,ix)=C(a(1:length(ix))); %可随机从C中提取长度为ix的数据
            end
        end
        
        %保证x中元素互不相等
        C=Compset(Y,x(i,:)); 
        if length(C)>S1-D
            [b,m]=unique(x(i,:),'first');
            Y1=[1:D];
            m_com=Compset(Y1,m);
            randnum=randperm(length(C)); %随机产生矩阵位置
            x(i,m_com)=C(randnum(1:length(m_com))); %可随机从C中提取数据
        end
        
        if fitness(x(i,:),D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor)<p(i)
            p(i)=fitness(x(i,:),D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor); 
            y(i,:)=x(i,:);
        end
        if p(i)<fitness(gbest,D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor);
            gbest=y(i,:);
        end
    end
end

%------显示计算结果
disp('*************************************************************')
disp('函数的全局最优位置为：')
Solution=gbest

disp('最后得到的优化极值为：')
[Result1,Result2,Result3,E_com1,E_pro1]=fitness(gbest,D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor);
Result=Result1
%---------------Result2:分簇信息
B=Result3;      %unchosen programmable sensor nodes

%更新节点消耗能量信息
E_com=E_com1;
E_pro=E_pro1;
disp('*************************************************************')

figure(2)
plot(coor_common(1,:),coor_common(2,:),'or');hold on;
plot(coor_pro(1,:),coor_pro(2,:),'*b');hold on;
plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on;
plot(coor_con(1,:),coor_con(2,:),'hk');hold on;
plot(50,50,'+m');
%连线
for i=1:D    
    for j=1:N
        if Result2(i,j)>S2               %nonchosen programmable sensor nodes
            line([coor_pro(1,B(Result2(i,j)-S2)),coor_pro(1,gbest(i))],[coor_pro(2,B(Result2(i,j)-S2)),coor_pro(2,gbest(i))]);
        elseif Result2(i,j)~=0               %common sensor nodes
            line([coor_common(1,Result2(i,j)),coor_pro(1,gbest(i))],[coor_common(2,Result2(i,j)),coor_pro(2,gbest(i))]);
        end
    end
end
legend('common sensor nodes','programmable sensor nodes','head nodes','programmable controller nodes','controller server');
title('head node selection and cluster formulation');hold off;
