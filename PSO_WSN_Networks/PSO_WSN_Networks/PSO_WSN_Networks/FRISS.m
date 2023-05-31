%#################################################################################################################################################################################
%This MATLAB code was used in paper "Particle swarm optimization implementation for minimal transmission power providing a fully-connected cluster for the Internet of Things"
%Access to paper: http://ieeexplore.ieee.org/abstract/document/7224573/
%Authors: Gabriel Lobão, Felipe Reis, Jonathan de Carvalho and Lucas Mendes
%#################################################################################################################################################################################

% friss(Gt, Gr, freq, Txp[1,2], Rxp[1,2])
% 
% Gt -> Ganho da antena transmissora
% Gr -> Ganho da antena receptora
% Freq -> frequencia em MHz
% Txp[x,y] -> Vetor posição do nó TRANSMISSOR
% Rxp[x,y] -> Vetor posição do nó RECEPTOR



function [ATTdb] = friss(Gt, Gr, freq, Txp, Rxp)

c = (3.*10.^8);
lambda = c ./ (freq.*10.^6);

% calculo da distancia entre transmissor e receptor
dist = (((Rxp(1) - Txp(1)).^2) + ((Rxp(2) - Txp(2)).^2)).^0.5;

ATT = Gt.*Gr.*(lambda./(4.*pi.*dist)).^2;
ATTdb = 10.*log10(ATT);
end
