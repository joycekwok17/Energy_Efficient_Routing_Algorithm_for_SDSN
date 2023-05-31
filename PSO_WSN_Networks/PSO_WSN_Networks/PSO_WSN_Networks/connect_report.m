%#################################################################################################################################################################################
%This MATLAB code was used in paper "Particle swarm optimization implementation for minimal transmission power providing a fully-connected cluster for the Internet of Things"
%Access to paper: http://ieeexplore.ieee.org/abstract/document/7224573/
%Authors: Gabriel Lobão, Felipe Reis, Jonathan de Carvalho and Lucas Mendes
%#################################################################################################################################################################################

function [soma] = connect_report(X,power_vector)

% X - matriz com as posições de cada nó
% power_vector - vetor com as potencias que cada nó está transmitindo
% 
% power_vector = [ P1  P2 P3 ... PN ]

N = size(X,2);          % Numero de Nós

% N = 20;
freq = 915;             % frequencia fixada em 915MHz

    %--------------------%
    %  Cálculo do custo  % 
    %--------------------%
    
cost = zeros(N);        % matriz de N-por-N posições

for l=1:N
    for c=1:N
        
        if l == c
            cost(l,c) = -Inf;
        else
            cost(l,c) = friss(1,1,freq, X(:,l), X(:,c));
        end
        
    end
end

% criando um power vector para rodar o código "em casa"

%   for i = 1:N
% %      power_vector(i) = (randi(50,1,1)/10)-10.5;
%       power_vector(i) = -20.371;
% % 
%   end
    %-------------------------------------------------%
    %     Convertendo power_vector de dBm p/ Watt     % 
    %-------------------------------------------------%

for i = 1:N
    power_vector_watt (i) = 10.^((power_vector(i)./10)-3);
end


    %-------------------------------------------------%
    % Contabilizando a potencia recebida por cada nó  % 
    %-------------------------------------------------%
% Esta rotina calcula a potencia recebida por cada nó (coluna) quando o
% transmissor é o nó l (linha)
PR = zeros(N);          % PR - matriz de potencia recebida.

for l = 1: N
    for c = 1 : N
        PR(l,c) = power_vector(l) + cost(l,c);
    end
end
    %-----------------------------------------%
    % Analisando se há existencia de conexão  % 
    %-----------------------------------------%
A = zeros(N);      % Matriz adjacencia
for l = 1:N
    for c = 1: N
        
        if PR(l,c) >= -60 
            A (l,c) = 1;
        else
            A (l,c) = 0;
        end
    end
end

for i=1:N
    D(i) = sum(A(i,:));
end
% A matriz status2, traz em sua diagonal principal o número de nós
% conectado ao nó indicado por status2(i,i). Se status2(i,i) for igual a
% ZERO, então não há vizinhos para o nó i.

% Se houver ao menos um nó desconectado, isto é, D(i,i) = 0, já é
% suficiente para afirmar que o grafo é desconexo.

connect = 1;
for i = 1:N
    connect = D(i).*connect;
end

% Entretanto, mesmo que haja pelo menos uma conexão por nó, não é possível
% afirmar que o grafo é conexo. Para isso é preciso respeitar a afirmação
% de de Miroslav Fiedler.

if connect == 0
    soma = Inf;

else

    % Determinando a matriz Laplaceana de A
    for i = 1:N
        for j = 1:N

            if i == j
                L(i,j) = D(i);
            else
                if  A(i,j) == 1
                    L(i,j) = -1;
                else

                L(i,j) = 0;
                end
            end
        end
    end

    autovetor = eig(L);
    
    if autovetor(2) > 0
        soma = 10.*log10((sum(power_vector_watt))/(1e-3));
    else
        soma = Inf;
    end
end

end 