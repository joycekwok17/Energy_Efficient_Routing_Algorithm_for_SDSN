%#################################################################################################################################################################################
%This MATLAB code was used in paper "Particle swarm optimization implementation for minimal transmission power providing a fully-connected cluster for the Internet of Things"
%Access to paper: http://ieeexplore.ieee.org/abstract/document/7224573/
%Authors: Gabriel Lob�o, Felipe Reis, Jonathan de Carvalho and Lucas Mendes
%#################################################################################################################################################################################

function [soma] = connect_report(X,power_vector)

% X - matriz com as posi��es de cada n�
% power_vector - vetor com as potencias que cada n� est� transmitindo
% 
% power_vector = [ P1  P2 P3 ... PN ]

N = size(X,2);          % Numero de N�s

% N = 20;
freq = 915;             % frequencia fixada em 915MHz

    %--------------------%
    %  C�lculo do custo  % 
    %--------------------%
    
cost = zeros(N);        % matriz de N-por-N posi��es

for l=1:N
    for c=1:N
        
        if l == c
            cost(l,c) = -Inf;
        else
            cost(l,c) = friss(1,1,freq, X(:,l), X(:,c));
        end
        
    end
end

% criando um power vector para rodar o c�digo "em casa"

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
    % Contabilizando a potencia recebida por cada n�  % 
    %-------------------------------------------------%
% Esta rotina calcula a potencia recebida por cada n� (coluna) quando o
% transmissor � o n� l (linha)
PR = zeros(N);          % PR - matriz de potencia recebida.

for l = 1: N
    for c = 1 : N
        PR(l,c) = power_vector(l) + cost(l,c);
    end
end
    %-----------------------------------------%
    % Analisando se h� existencia de conex�o  % 
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
% A matriz status2, traz em sua diagonal principal o n�mero de n�s
% conectado ao n� indicado por status2(i,i). Se status2(i,i) for igual a
% ZERO, ent�o n�o h� vizinhos para o n� i.

% Se houver ao menos um n� desconectado, isto �, D(i,i) = 0, j� �
% suficiente para afirmar que o grafo � desconexo.

connect = 1;
for i = 1:N
    connect = D(i).*connect;
end

% Entretanto, mesmo que haja pelo menos uma conex�o por n�, n�o � poss�vel
% afirmar que o grafo � conexo. Para isso � preciso respeitar a afirma��o
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