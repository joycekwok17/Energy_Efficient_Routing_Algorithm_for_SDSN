function myrouting(Solution,coor_pro,E_pro_node,xc,Y,D)
global rest_pro;
rest_pro=setdiff(Y,Solution);
global CN;
d0=10;           % single hop dis threshold
single_hop_set=[];
multi_hop_set=[];
figure(3)
plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on; % head node selection
plot(xc(1,:),xc(2,:),'hk');hold on;
ju_min(1:D)=10000;
s=0;
for i=1:4
     for j=1:D
          if ismember(j,single_hop_set)~=1 %    j����single set��
            ju=sqrt((coor_pro(1,Solution(j))-xc(1,i)).^2+(coor_pro(2,Solution(j))-xc(2,i)).^2);
            if ju<ju_min(j)
                ju_min(j)=ju;
                ju_con(j)=i;          
            end
          end  
     end
end

 % �ҳ���������
for j=1:D
    if ju_min(j)<d0
        single_hop_set=[single_hop_set,j];     
        line([coor_pro(1,Solution(j)),xc(1,ju_con(j))], ...
            [coor_pro(2,Solution(j)),xc(2,ju_con(j))]);hold on; % ���ߣ������ڵ㵽��Ӧ���ƽڵ�
%               path(j)=[];
        cost(j)=0;
        s=s+1; % �����ɵ��ڵ����
    end
end

% ��ʼ��Ȩֵ����link weight matrix
weight=zeros(D,D);
a=0.5;
for i=1:D         % start node
    if ismember(i,single_hop_set)==1
        weight(i,:)=inf;            % �������ɵ���start�ڵ�Ȩֵ����Ϊ�����
    else
        for j=1:D % destination node
            ju=sqrt((coor_pro(1,Solution(i))-coor_pro(1,Solution(j))).^2 ...
                +(coor_pro(2,Solution(i))-coor_pro(2,Solution(j))).^2);
            if i==j
                weight(i,j)=inf;
            elseif ju<d0
                weight(i,j)=a*ju*ju/E_pro_node(Solution(j))+(1-a)*CN(j); 
            else 
                weight(i,j)=a*ju*ju*ju*ju/E_pro_node(Solution(j))+(1-a)*CN(j);
            end
        end
    end
end

% child=zeros(1,D);% ��¼ÿ��ͷ�ڵ���Ϊ�м�ڵ㣬�����ӽڵ��packet����

% Dijkstra �㷨
t=1;
while (t<(D-s+1)) %  
    for j=1:D
        if (ismember(j,single_hop_set)==1)&& (ismember(j,multi_hop_set)==1)
            weight(j,:)=inf;
        end
    end
    valid_weight=weight(:,single_hop_set);
    weight_min=min(min(valid_weight));
    [index_start,index_dest]=find(weight==weight_min) ;        % x����,D�е����
    line([coor_pro(1,Solution(index_start(1))),coor_pro(1,Solution(index_dest(1)))],...
          [coor_pro(2,Solution(index_start(1))),coor_pro(2,Solution(index_dest(1)))]);hold on;  % start��dest node����
    single_hop_set=[single_hop_set,index_start(1)];    % current index ����single hopset 
    multi_hop_set=[multi_hop_set,index_start(1)] ;      % current index ����multi hopset 
%     child_packet(index_dest)=child_packet(index_dest)+CN(index_start);% ��Ϊ�м�ڵ�
%     path(index_start)=[path(index_dest),index_dest]; % ·��
%     cost(index_start)=weight_min+cost(index_dest);   % ·���ܻ��� 
    weight(index_start(1),:)=inf;  % index��Ϊstart����Ȩֵ�����н���һ�е�ֵ����Ϊ�����
    t=t+1;
end
end    
      

    

 
        
    
            
        
            
              
