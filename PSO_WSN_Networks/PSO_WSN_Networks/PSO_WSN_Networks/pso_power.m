%#################################################################################################################################################################################
%This MATLAB code was used in paper "Particle swarm optimization implementation for minimal transmission power providing a fully-connected cluster for the Internet of Things"
%It was based on this following code: http://www.mathworks.com/matlabcentral/fileexchange/20205-particle-swarm-optimization/content/PSO_Wael/PSO/PSO.m.
%Access to paper: http://ieeexplore.ieee.org/abstract/document/7224573/
%This code implements a PSO algorithm to optimize transmission powers for each node in an WSN networks
%Authors: Gabriel Lobão, Felipe Reis, Jonathan de Carvalho and Lucas Mendes
%#################################################################################################################################################################################

clear all;

for cen=1:10
    
cenario = strcat('space',int2str(cen));
load(cenario)
nomeArquivo = strcat(cenario,'.t2');

fid = fopen(nomeArquivo,'wt');
fprintf(fid,'##################Início do Treinamento##################\n\n');
fprintf(fid,'Cenario = %s\n\n',cenario);


for t=1:5
tic
fprintf(fid,'##########################################################################\n');
fprintf(fid,'                           TESTE = %d                                    \n',t);
fprintf(fid,'##########################################################################\n');

n = 30;          % tamanho do enxame %%·äÈº´óÐ¡
num_passos = 500; % máximo número de passos
dim = size(X,2);   % dimensão da partícula

wmax=0.9;  %fator de inércia inicial
wmin=0.4;  %fator de inércia final
c1=2;      %coeficiente de aceleração
c2=2;      %coeficiente de aceleração

%Define o limite do espaço de busca
pMin = -30;
pMax = 0;

%Define o limite da velocidade
vMax = 5; 
vMin = -vMax;

fprintf(fid,'Configuracoes do PSO\n');
fprintf(fid,'Numero de particulas = %f\n',n);
fprintf(fid,'Numero maximo de passos = %f\n',num_passos);
fprintf(fid,'Dimensao do problema = %f\n',dim);
fprintf(fid,'Coeficiente C1 = %f\n',c1);
fprintf(fid,'Coeficiente C2 = %f\n',c2);
fprintf(fid,'Inercia (wmax) = %f / (wmin) = %f \n\n',wmax,wmin);
fprintf(fid,'pMax = %f / pMin = %f\n',pMax,pMin);
fprintf(fid,'vMax = %f / vMin = %f\n',vMax,vMin);

fitness=0*ones(n,num_passos);                           
current_fitness =0*ones(n,1);



%Inicialização das posições das partículas
vetor_potencia = pMin + (pMax - pMin)*rand(dim,n);                   

%Inicialização da velocidade da partícula
velocidade = rand(dim,n); 
%velocidade = vMin + (vMax - vMin)*rand(dim,n);


%Calcula fitness inicial das partículas                                
for i=1:n
        current_fitness(i) = connect_report(X,vetor_potencia(:,i)) ;    
end


%Inicializa o PBEST
pbest_fitness  = current_fitness ;
pbest_position  = vetor_potencia ;

%Inicializa o GBEST
[gbest_fitness,g] = min(pbest_fitness) ;
for i=1:n
    gbest_position(:,i) = pbest_position(:,g) ;
end

for iter=1:num_passos   

%Decrementa linearmente o fator de inercia
w = wmax - ((wmax-wmin)/num_passos)*iter; 

%Calcula nova velocidade da particula
for i=1:n
    velocidade(:,i) = w*velocidade(:,i) + c1*(rand*(pbest_position(:,i)-vetor_potencia(:,i))) + c2*(rand*(gbest_position(:,i)-vetor_potencia(:,i)));
end

%Limita a velocidade da particula
for k=1:n
    index = velocidade(:,k)>vMax;
    velocidade(index,k)=vMax;

    index = velocidade(:,k)<vMin;
    velocidade(index,k)=vMin;
end	

%Calcula nova posicação da particula
vetor_potencia = vetor_potencia + velocidade;

%Limita o espaço de busca da particula
for k=1:n         
    index = vetor_potencia(:,k)>pMax;
    vetor_potencia(index,k)=pMax;

    index = vetor_potencia(:,k)<pMin;
    vetor_potencia(index,k)=pMin;
end



%Avalia novo fitness da partícula
for i=1:n,
        current_fitness(i) = connect_report(X,vetor_potencia(:,i)) ;    
end

%Calcula pbest
for i=1:n
    if current_fitness(i) < pbest_fitness(i)
        pbest_fitness(i)  = current_fitness(i);  
        pbest_position(:,i) = vetor_potencia(:,i);
    end
end

[current_gbest_fitness,g] = min(pbest_fitness);

%Calcula gbest
if current_gbest_fitness < gbest_fitness
    gbest_fitness = current_gbest_fitness;
    for i=1:n
        gbest_position(:,i) = pbest_position(:,g);       
    end

end 

 if(iter==1 || rem(iter,50)==0)  
   sprintf('Iteracao - %4d | Soma = %f\n',iter,gbest_fitness)
   fprintf(fid,'Iteracao - %4d | Soma = %f\n',iter,gbest_fitness);
 end
 
end %fim do algoritmo
 
gbest_fitness;
d=gbest_position(:,1);

fprintf(fid,'\n\n');
fprintf(fid,'Potência de cada nó\n\n');
for i=1:dim    
 	fprintf(fid,'N?%i= %f ',i, d(i));	   
    fprintf(fid,'\n');
end
fprintf(fid,'\n\n');
 

tempoGasto = toc;
fprintf(fid,'Tempo gasto: %d\n\n',tempoGasto);

end %fim for t=1:10

fclose(fid);

end
