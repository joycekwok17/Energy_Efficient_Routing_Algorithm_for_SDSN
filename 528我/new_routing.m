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
%   找出单跳集合
    s=0;
    for j=1:D
            if ismember(j,single_hop_set)~=1      % j不在single set中
                ju=sqrt((coor_pro(1,Solution(j))-xc).^2+(coor_pro(2,Solution(j))-yc).^2);
                if ju<D0
                    single_hop_set=[single_hop_set,j];
                    line([coor_pro(1,Solution(j)),xc], ...
                           [coor_pro(2,Solution(j)),yc],'Color','red');hold on; % 连红线，单跳节点到控制节点
                    s=s+1;               % s:单跳可到节点个数
                end
            end  
    end

% 初始化权值矩阵link weight matrix
    weight=zeros(D,D);
    gama=0.5;
    for i=1:D         % start node
        if ismember(i,single_hop_set)==1
            weight(i,:)=inf;            % 将单跳可到的start节点权值设置为无穷大
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

% Dijkstra 算法
child=zeros(1,D);   % 记录每个头节点作为中间节点，接收上一跳节点的数据包数量，子节点个数
parent=zeros(1,D);
judge=zeros(D,D);   %行代表start，列代表dest，判断dest是start的parent
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
    [index_start,index_dest]=find(weight==weight_min) ;         % x坐标,D中的序号
    
%     child(index_dest)=child(index_dest)+CN(index_start(1));   
    child(index_dest)=child(index_dest)+1;   % dest node要接受来自start node的包
    judge(index_start(1),index_dest)=1;
    parent(index_start(1))=parent(index_start(1))+1;    %start要向dest节点发送的包

    
    line([coor_pro(1,Solution(index_start(1))),coor_pro(1,Solution(index_dest(1)))],...
          [coor_pro(2,Solution(index_start(1))),coor_pro(2,Solution(index_dest(1)))]);hold off;  % start与dest node连线
    
    single_hop_set=[single_hop_set,index_start(1)];    % current index 加入single hopset 
    multi_hop_set=[multi_hop_set,index_start(1)] ;      % current index 加入multi hopset 
    weight(index_start(1),:)=inf;  % index作为start，在权值矩阵中将这一行的值设置为无穷大
    t=t+1;
end
end

      

    

 
        
    
            
        
            
              
