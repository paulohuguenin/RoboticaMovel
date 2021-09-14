function J = obstaclefunction(x,w1,obstacles) 


sigma =1.5;
%w1=10.5%
%r = 2.25; % 1.5

[m,n] = size(obstacles);
dist = x*ones(1,n)-obstacles; % Multiplicar matrizes 2 * 1 e 1 * n é equivalente à expansão equivalente
dist = sum(dist.^2); %Adicionar quadrados de coluna
J = min(dist); 
%if J >= 0.2
     J = w1*5.5*exp(-sigma*J); %Use expoente como função de campo potencial
%else 
%    J = w1*4.5*exp(-sigma*0.2);
%end