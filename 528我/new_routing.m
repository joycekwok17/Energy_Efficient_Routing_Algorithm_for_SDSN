function [child,judge,parent]=new_routing(Solution,coor_pro,E_pro_node,Y,D,xc,yc,d0)
    global rest_pro;
    rest_pro=setdiff(Y,Solution);
    global CN;
    D0=50;           % single hop distance threshold
    single_hop_set=[];
    multi_hop_set=[];

    figure(2)
    plot(coor_pro(1,Solution),coor_pro(2,Solution),'*g');hold on; % head node selection
    plot(xc,yc,'hk');hold on;

%   ju_min(1:D)=10000;
%   �ҳ���������
    s=0;
    for j=1:D
            if ismember(j,single_hop_set)~=1      % j����single set��
                ju=sqrt((coor_pro(1,Solution(j))-xc).^2+(coor_pro(2,Solution(j))-yc).^2);
                if ju<D0
                    single_hop_set=[single_hop_set,j];
                    line([coor_pro(1,Solution(j)),xc], ...
                           [coor_pro(2,Solution(j)),yc],'Color','red');hold on; % �����ߣ������ڵ㵽���ƽڵ�
                    s=s+1;               % s:�����ɵ��ڵ����
                end
            end  
    end

% ��ʼ��Ȩֵ����link weight matrix
    weight=zeros(D,D);
    gama=0.5;
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
                    weight(i,j)=gama*ju*ju/E_pro_node(Solution(j))+(1-gama)*CN(j); 
                else 
                    weight(i,j)=gama*ju*ju*ju*ju/E_pro_node(Solution(j))+(1-gama)*CN(j);
                end
            end
        end
    end

% Dijkstra �㷨
child=zeros(1,D);   % ��¼ÿ��ͷ�ڵ���Ϊ�м�ڵ㣬������һ���ڵ�����ݰ��������ӽڵ����
parent=zeros(1,D);
judge=zeros(D,D);   %�д���start���д���dest���ж�dest��start��parent
% parent=[];
t=1;
while (t<(D-s+1)) %  
    for j=1:D
        if (ismember(j,single_hop_set)==1)&& (ismember(j,multi_hop_set)==1)
            weight(j,:)=inf;
        end
    end
    valid_weight=weight(:,single_hop_set);
    weight_min=min(min(valid_weight));
    [index_start,index_dest]=find(weight==weight_min) ;         % x����,D�е����
    
%     child(index_dest)=child(index_dest)+CN(index_start(1));   
    child(index_dest)=child(index_dest)+1;   % dest nodeҪ��������start node�İ�
    judge(index_start(1),index_dest)=1;
    parent(index_start(1))=parent(index_start(1))+1;    %startҪ��dest�ڵ㷢�͵İ�

    
    line([coor_pro(1,Solution(index_start(1))),coor_pro(1,Solution(index_dest(1)))],...
          [coor_pro(2,Solution(index_start(1))),coor_pro(2,Solution(index_dest(1)))]);hold off;  % start��dest node����
    
    single_hop_set=[single_hop_set,index_start(1)];    % current index ����single hopset 
    multi_hop_set=[multi_hop_set,index_start(1)] ;      % current index ����multi hopset 
    weight(index_start(1),:)=inf;  % index��Ϊstart����Ȩֵ�����н���һ�е�ֵ����Ϊ�����
    t=t+1;
end
end

      

    

 
        
    
            
        
            
              
