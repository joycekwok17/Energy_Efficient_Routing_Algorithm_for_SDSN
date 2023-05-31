% NWPSO-based SDWSN energy-efficient routing algorithm simulation
clc;
clear;
close all;
% node deployment and cluster formation
%% parameter initialization
  % 能量模型初始化数据
    E1=0.5; %普通传感节点初始能量
    E2=2 ;  %SDSN传感节点初始能量
    E3=10;  %SDSN控制节点初始能量
    Elec=50e-9;     %发送、接收能量，每bit  
    Efs=10e-12;     %耗散能量，每bit
    Emp=0.0013e-13; %融合能耗，每bit
    d0 = sqrt( Efs/Emp );%传输距离阈值
    P=4000;%单位时间内数据包大小
    Rmax=5;%普通节点传输距离
   
    %生成初始节点
    xm=100; 
    ym=100; % 传感区域范围
    n1=100; %普通传感器节点数
    n2=40;  %SDSN传感器节点数
    n3=4;   %控制节点数
    xs=0;%控制器位于原点
    ys=0;
    xc1=-5;%4 controll nodes
    yc1=-5;
    xc2=-5;
    yc2=5;
    xc3=5;
    yc3=5;
    xc4=5;
    yc4=-5;
    x1=-xm/2+xm*rand(1,n1);% 随机生成n1个普通传感器节点
    y1=-ym/2+ym*rand(1,n1);
    x2=-xm/2+xm*rand(1,n2);% 随机生成n2个SDSN传感器节点
    y2=-ym/2+ym*rand(1,n2);
    figure(1);
    scatter(x1,y1,'b');
    hold on;
    scatter(x2,y2,'r*');
    hold on;
    plot(xs,ys,'gh',xc1,yc1,'y+:',xc2,yc2,'y+:',xc3,yc3,'y+:',xc4,yc4,'y+:');
    hold on;
    
    
%% PSO Problem Definition
function z=Sphere(x)
    z=sum(x.^2);
end

CostFunction=@(x) Sphere(x);        % Cost Function

nVar=10;            % Number of Decision Variables %%粒子维数

VarSize=[1 nVar];   % Size of Decision Variables Matrix

VarMin=-10;         % Lower Bound of Variables
VarMax= 10;         % Upper Bound of Variables


%% PSO Parameters

MaxIt=50;      % Maximum Number of Iterations

nPop=100;        % Population Size (Swarm Size)

% PSO Parameters
w=1;            % Inertia Weight
wdamp=0.99;     % Inertia Weight Damping Ratio
c1=1.5;         % Personal Learning Coefficient
c2=2.0;         % Global Learning Coefficient

% If you would like to use Constriction Coefficients for PSO,
% uncomment the following block and comment the above set of parameters.

% % Constriction Coefficients
% phi1=2.05;
% phi2=2.05;
% phi=phi1+phi2;
% chi=2/(phi-2+sqrt(phi^2-4*phi));
% w=chi;          % Inertia Weight
% wdamp=1;        % Inertia Weight Damping Ratio
% c1=chi*phi1;    % Personal Learning Coefficient
% c2=chi*phi2;    % Global Learning Coefficient

% Velocity Limits
VelMax=0.1*(VarMax-VarMin);
VelMin=-VelMax;

%% Initialization

empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

particle=repmat(empty_particle,nPop,1);

GlobalBest.Cost=inf;

for i=1:nPop
    
    % Initialize Position
    particle(i).Position=unifrnd(VarMin,VarMax,VarSize);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    % Evaluation
    particle(i).Cost=CostFunction(particle(i).Position);
    
    % Update Personal Best
    particle(i).Best.Position=particle(i).Position;
    particle(i).Best.Cost=particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest=particle(i).Best;
        
    end
    
end

BestCost=zeros(MaxIt,1);

%% PSO Main Loop

for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity,VelMin);
        particle(i).Velocity = min(particle(i).Velocity,VelMax);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Velocity Mirror Effect
        IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
        particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);
        
        % Apply Position Limits
        particle(i).Position = max(particle(i).Position,VarMin);
        particle(i).Position = min(particle(i).Position,VarMax);
        
        % Evaluation
        particle(i).Cost = CostFunction(particle(i).Position);
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost
                
                GlobalBest=particle(i).Best;
                
            end
            
        end
        
    end
    
    BestCost(it)=GlobalBest.Cost;
    
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
 
    
    w=w*wdamp;
    
end

BestSol = GlobalBest;

%% Results

figure;
% plot(BestCost,'LineWidth',2);
semilogy(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
