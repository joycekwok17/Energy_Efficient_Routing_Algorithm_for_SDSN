function [E_pro,E_com,Solution]=fpso(D,M,N,S1,S2,E_pro_sensor,E_com_sensor,coor_common,coor_pro,coor_con)
% S1:pro sensor node������
% S2��com sensor node������
% D����ͷ������
% M�����Ӹ�����
% N��sensor node����
%------������ʼ������----------------------------------------------
c1=2;             %ѧϰ����1
c2=2;             %ѧϰ����2
w=0.7;            %����Ȩ��
MaxDT=1000;       %����������
% eps=10^(-6);      %���þ���(����֪��Сֵʱ����)

aaa=1;
for i=1:S1
    if E_pro_sensor(i)>0
        Y(aaa)=i;           % YΪ���pro sensor node���
        aaa=aaa+1;
    end
end

Y1=[1:S1];

x=zeros(M,D);     %%
v=zeros(M,D);

%------��ʼ����Ⱥ�ĸ���(�����������޶�λ�ú��ٶȵķ�Χ)------------
for i=1:M
    PA=randperm(length(Y));
    x(i,:)=Y(PA(1:D)); 
    for j=1:D
        v(i,j)=randn;
    end
end
            
%------�ȼ���������ӵ���Ӧ�ȣ�����ʼ��p(i)��gbest--------------------
for i=1:M
    p(i)=fitness(x(i,:),D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor);   
    y(i,:)=x(i,:);
end

gbest=x(1,:);             %gbestΪȫ�����ţ���ʼ��gbest
for i=2:M
    if fitness(x(i,:),D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor) > fitness(gbest,D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor)
        gbest=x(i,:);
    end
end

%------������Ҫѭ�������չ�ʽ���ε�����ֱ�����㾫��Ҫ��------------
for t=1:MaxDT
    for i=1:M
        v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(gbest-x(i,:));
        x(i,:)=mod(round(x(i,:)+v(i,:)),S1);
        for j=1:D
            if x(i,j)==0;
               x(i,j)=1;
            end            
        end
        
        %��֤x�в����������ڵ�
        DEAD=Compset(Y1,Y);       %�����ڵ����
        if isempty(DEAD)~=1       %DEAD��Ϊ��
            [inter,ix,id]=intersect(x(i,:),DEAD); %interΪ����Ԫ�أ�ix��ʾ����Ԫ����x�е�λ�ã�id��ʾ����Ԫ����DEAD��λ�á�
            C=Compset(Y,x(i,:)); 
            if isempty(inter)~=1
                a=randperm(length(C)); %�����������λ��
                x(i,ix)=C(a(1:length(ix))); %�������C����ȡ����Ϊix������
            end
        end
        
        %��֤x��Ԫ�ػ������
        C=Compset(Y,x(i,:)); 
        if length(C)>S1-D
            [b,m]=unique(x(i,:),'first');
            Y1=[1:D];
            m_com=Compset(Y1,m);
            randnum=randperm(length(C)); %�����������λ��
            x(i,m_com)=C(randnum(1:length(m_com))); %�������C����ȡ����
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

%------��ʾ������
disp('*************************************************************')
disp('������ȫ������λ��Ϊ��')
Solution=gbest

disp('���õ����Ż���ֵΪ��')
[Result1,Result2,Result3,E_com1,E_pro1]=fitness(gbest,D,N,S1,S2,Y,coor_common,coor_pro,E_com_sensor,E_pro_sensor);
Result=Result1
%---------------Result2:�ִ���Ϣ
B=Result3;      %unchosen programmable sensor nodes

%���½ڵ�����������Ϣ
E_com=E_com1;
E_pro=E_pro1;
disp('*************************************************************')

figure(2)
plot(coor_common(1,:),coor_common(2,:),'or');hold on;
plot(coor_pro(1,:),coor_pro(2,:),'*b');hold on;
plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on;
plot(coor_con(1,:),coor_con(2,:),'hk');hold on;
plot(50,50,'+m');
%����
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
