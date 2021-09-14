% 
% Método de campo de potencial artificial para planejamento do caminho do robô 

clear
close all; 

a1 = [2, 6];
a2 = [4, 6];
a3 = [3, 3];

b1 = [5, 8];
b2 = [7, 8];
b3 = [8, 7];
b4 = [7, 5];
b5 = [5, 6];

c1 = [6, 4];
c2 = [8, 2];
c3 = [5, 2];

i1 = [1;3];
i2 = [1;6];

f1 = [9;8];
f2 = [9;6];
f3 = [9;3];

%{
i1 = [2;4];
i2 = [2; 6];

f1 = [13; 12];
f2 = [13; 12];
f3 = [13; 3];
%}
A = polyshape([a1; a2; a3]);
B = polyshape([b1; b2; b3; b4; b5]);
C = polyshape([c1; c2; c3]);

[centro_A_x, centro_A_y] = centroid(A);
[centro_B_x, centro_B_y] = centroid(B);
[centro_C_x, centro_C_y] = centroid(C);

% test scale
figure,
plot([A, B, C],'HandleVisibility','off')
hold on
plot(i1(1),i1(2),'sb','LineWidth',6,'HandleVisibility','off')
plot(i2(1),i2(2),'sb','LineWidth',6,'HandleVisibility','off')
plot(f1(1),f1(2),'sr','LineWidth',6,'HandleVisibility','off')
plot(f2(1),f2(2),'sr','LineWidth',6,'HandleVisibility','off')
plot(f3(1),f3(2),'sr','LineWidth',6,'HandleVisibility','off')
title('Ambiente');
xlabel('x'); 
ylabel('y');
axis([0 10 0 10])
%hold off

 n=3;
        
        obsa1=genObsPoly(a1,a2,n);
        obsa2=genObsPoly(a1,a3,n);
        obsa3=genObsPoly(a2,a3,n);
        
        obsb1=genObsPoly(b1,b2,n);
        obsb2=genObsPoly(b2,b3,n);
        obsb3=genObsPoly(b3,b4,n);
        obsb4=genObsPoly(b4,b5,n);
        obsb5=genObsPoly(b5,b1,n);
        
        obsc1=genObsPoly(c1,c2,n);
        obsc2=genObsPoly(c1,c3,n);
        obsc3=genObsPoly(c2,c3,n);
        
        
        
        %obs=cat(1,[a1,a2,a3,b1,b2,b3,b4,b5,c1,c2,c3,obsa1,obsa2,obsa3,obsb1,obsb2,obsb3,obsb4,obsb5,obsc1,obsc2,obsc3]);
        obs=[a1;a2;a3;b1;b2;b3;b4;b5;c1;c2;c3;obsa1;obsa2;obsa3;obsb1;obsb2;obsb3;obsb4;obsb5;obsc1;obsc2;obsc3;...
            ];
        %obs=[[3,4.7];[6,3];[6.5,6];];
%figure
for j=[f3]
for i=[i2]
% Definir Área de trabalho
xmin = [0; 0];  
xmax = [10;10];

% Maximum number
Nsteps = 600;

% Defina os parâmetros do robô
%Parâmetro de comprimento do passo de movimento do robô% na direção selecionada
lambda = 0.1;
Ns=30; 
r = 1; 
xs=0*ones(2,Ns); 
Jo(:,1)=0*ones(Ns,1); 
Jg(:,1)=0*ones(Ns,1); 
J(:,1)=0*ones(Ns,1); 
theta(:,1)=0*ones(Ns,1);


for m=2:Ns
    theta(m,1)=theta(m-1,1)+(pi/180)*(360/Ns); 
end 
%theta=linspace(0,pi/2,30)';
% Defina as coordenadas(Goal/Target)
P_Goal=j;%[13;6];
obstacles =  vertcat(obs(:,1)',obs(:,2)'); %[6 20 11 16 18 19 ;6 16 17 14 11.9 19];
Mat = size(obstacles); %Pontos de obstáculos
obNum = Mat(1,2);
nt = 20; % Etapas do movimento
nr = 20; % RoA velocidade determina se você pode acompanhar
x1 = 1;
y1 = 1;
g = 1;
h = 0;
distrt = 0; % Calcular distância, condição final
distro = 0*ones(2,obNum); % Calcule a distância, evite críticas
t = 0;
na = 0;

% Defina as coordenadas da posição inicial do robô
P_Ro=i;%[2;6]; 
w1 = 1.5; 
w2 =2; 
P_Ro(:,2:Nsteps) = 0*ones(2,Nsteps-1);

%Desenhar campo potencial

xx=0:20/100:20; 
yy=xx; 

%Calcule a função potencial de obstáculo 
 for jj=1:length(xx) 
    for ii=1:length(yy) 
       op(ii,jj)=obstaclefunction([xx(jj);yy(ii)],w1,obstacles); 
    end 
 end 
 
% Calcule a função potencial objetivo
 for jj=1:length(xx) 
    for ii=1:length(yy) 
        gp(ii,jj)=goalfunction([xx(jj);yy(ii)],P_Goal(:,1),w2); 
    end 
 end 
%}
P_RoA = P_Ro(1,1);
P_RoB = P_Ro(2,1);
P_RoC = P_Ro(1,1);
P_RoD = P_Ro(2,1);

potential = gp - op;

figure;

plot(P_Goal(1,1),P_Goal(2,1),'+','MarkerSize',10);
hold on
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',15);

title('Potencial Total');
xlabel('x'); 
ylabel('y'); 

plot(obstacles(1,:),obstacles(2,:),'o', 'MarkerEdgeColor','k','MarkerSize',10); 
contour(xx,yy,potential,90);

axis([0 15 0 15]); 
hold off

figure,
subplot(1,2,1)
plot(P_Goal(1,1),P_Goal(2,1),'x','MarkerSize',10);
hold on
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);

xlabel('x'); 
ylabel('y');


hold on
plot(obstacles(1,:),obstacles(2,:),'o','MarkerSize',22);

plot(obstacles(1,:),obstacles(2,:),'o', 'MarkerEdgeColor','r','MarkerSize',10); 
 contour(xx,yy,op,20);
 axis([0 15 0 15]); 
hold off

subplot(1,2,2)
plot(P_Goal(1,1),P_Goal(2,1),'x','MarkerSize',10);
hold on
plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);

xlabel('x'); 
ylabel('y'); 
%plot(obstacles(1,:),obstacles(2,:),'o','MarkerSize',22); 
plot(obstacles(1,:),obstacles(2,:),'o', 'MarkerEdgeColor','b','MarkerSize',10); 
 contour(xx,yy,gp,50);
 axis([0 15 0 15]); 
hold off
%}
% Robot Motion path simulation process
P_Goal_1 = P_Goal(:,1);
cont=0;
for k=1:Nsteps
    cont=cont+1;
    %Set motion boundaries
    P_Ro(:,k) = min(P_Ro(:,k),xmax); 
    P_Ro(:,k) = max(P_Ro(:,k),xmin); 
    if k>5 & abs(P_Ro(:,k-5)-P_Ro(:,k))<=0.08
        obstacles(:,length(obstacles)+1)=P_Ro(:,k);
    end
        
        
        
        
    for m=1:Ns 
        xs(:,m) = [P_Ro(1,k)+r*cos(theta(m,1)); P_Ro(2,k)+r*sin(theta(m,1))];      
        % Calcule se deve entrar na gama de obstáculos
        for t = 1:obNum
        distro(:,t) = xs(:,m) - obstacles(:,t);
        end
        sum1 = sum(distro.^2);
        %if k>5
            
        if min(sum1) < 1.69 % Defina o valor quadrado da distância de ação do obstáculo
           Jo(m,1) = 100;
        else
           Jo(m,1) = obstaclefunction(xs(:,m),w1,obstacles); 
        end
        Jg(m,1) = goalfunction(xs(:,m),P_Goal_1,w2); 
        J(m,1)= Jg(m,1) - Jo(m,1);
    end 
    

    [val,num] = max(J); 
    
    
    th(cont,1)=theta(num,1);

    distrt = P_Ro(:,k) - P_Goal_1; 
    
    %d=sqrt((P_Ro(1,k)-P_Goal(1,1))^2 + (P_Ro(2,k)-P_Goal(2,1))^2);
    %custo= sqrt((d)^2 + 1*(th(num,1)-0.1)^2);
    
    
    if sum(distrt.^2) > 0.5 
    
    %if distrt>0.5
        P_Ro(:,k+1) = [P_Ro(1,k)+lambda*cos(theta(num,1)); P_Ro(2,k)+lambda*sin(theta(num,1))]; 
    else
        break;
    end

    P_RoA = P_Ro(1,k+1);
    P_RoB = P_Ro(2,k+1);
    P_RoC = P_Ro(1,1:k+1);
    P_RoD = P_Ro(2,1:k+1);
    Deltalambda=0.1*lambda*(2*rand-1); 
    Deltatheta=2*pi*(2*rand-1); 
    P_Ro(:,k+1)=[P_Ro(1,k+1)+Deltalambda*cos(theta(num,1)+Deltatheta); P_Ro(2,k+1)+Deltalambda*sin(theta(num,1)+Deltatheta)]; 
end

figure;
% ?????
plot(obstacles(1,:),obstacles(2,:),'o','MarkerSize',10,'HandleVisibility','off');
hold on;
plot([A, B, C],'HandleVisibility','off')
% ???????
plot(P_Ro(1,1:k) ,P_Ro(2,1:k) ,'r-','DisplayName','Trajetória APF'); 
hold on
plot(P_Ro(1,1),P_Ro(2,1),'s', 'MarkerFaceColor','b','MarkerSize',10,'HandleVisibility','off');
plot(P_Goal(1,1),P_Goal(2,1),'sr','MarkerSize',8,'HandleVisibility','off');
%plot(P_Goal(1,1),P_Goal(2,1),'o','MarkerSize',22);
axis([0 10 0 10]); 
xlabel('x'); 
ylabel('y'); 
title('Caminho percorrido por campos potenciais');
%plot([A, B, C])

path=reshape(nonzeros(P_Ro'),[],2);
startLocation=path(1,:);
bicycle = bicycleKinematics("VehicleInputs","VehicleSpeedHeadingRate","MaxSteeringAngle",pi/4,"WheelBase",0.25);
initPose = [startLocation'; 0]; % Initial pose (x y theta)
controller2 = controllerPurePursuit("Waypoints",path,"DesiredLinearVelocity",0.5,"MaxAngularVelocity",pi,"LookaheadDistance",0.5);

goalPoints = path(end,:)';
goalPoints=vertcat(goalPoints,0);
goalRadius = 0.3;

sampleTime = 0.2;               % Sample time [s]
tVec = 0:sampleTime:50;
[tBicycle,bicyclePose] = ode45(@(t,y)derivative(bicycle,y,exMobileRobotController(controller2,y,goalPoints,goalRadius)),tVec,initPose);

bicycleTranslations = [bicyclePose(:,1:2) zeros(length(bicyclePose),1)];
bicycleRot = axang2quat([repmat([0 0 1],length(bicyclePose),1) bicyclePose(:,3)]);

plotTransforms(bicycleTranslations(1:15:end,:),bicycleRot(1:15:end,:),'MeshFilePath','groundvehicle.stl',"MeshColor","b","FrameSize",0.4);

plot(bicyclePose(:,1),bicyclePose(:,2),'DisplayName','Traj. Controlador');
legend('Location','northwest');
view(0,90)

end
end
tPRM=[20,19.6,18.8,19.4,19;16.4,16.6,16.6,16.2,16.6;16,16,16.4,16.4,16.8;16,15.6,18,17,16;16.4,16.6,16.2,16,16.2;16,17,16.2,18.4,16.8]
tAPF=[24.6,24.4,24.8,27,25.8;26,25.6,28.4,27,25.8;19.8,19.6,19.4,19.4,19.6;18.2,18,18.2,18.2,18.2;21.8,21.6,21.4,21.4,21.8 ; 25,25,24.8,24.8,24.8]