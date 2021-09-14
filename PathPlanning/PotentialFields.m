close all
clear all 
clc

a1 = [2+1, 6];
a2 = [4+1, 6];
a3 = [3+1, 3];

b1 = [5+3, 8+2];
b2 = [7+3, 8+2];
b3 = [8+3, 7+2];
b4 = [7+3, 5+2];
b5 = [5+3, 6+2];

c1 = [6+3, 4];
c2 = [8+3, 2];
c3 = [5+3, 2];

i1 = [2, 4];
i2 = [2, 6];

f1 = [13, 9];
f2 = [13, 6];
f3 = [13, 3];

A = polyshape([a1; a2; a3]);
B = polyshape([b1; b2; b3; b4; b5]);
C = polyshape([c1; c2; c3]);

[centro_A_x, centro_A_y] = centroid(A);
[centro_B_x, centro_B_y] = centroid(B);
[centro_C_x, centro_C_y] = centroid(C);

% test scale
plot([A, B, C])
hold on
%plot(i1(1),i1(2),'sb','LineWidth',6)
%plot(i2(1),i2(2),'sb','LineWidth',6)
%plot(f1(1),f1(2),'sr','LineWidth',6)
%plot(f2(1),f2(2),'sr','LineWidth',6)
%plot(f3(1),f3(2),'sr','LineWidth',6)
title('Ambiente');
xlabel('x'); 
ylabel('y');
axis([0 15 0 15])
%hold off
%',...
 %   'LineWidth',2,...
  %  'MarkerSize',10,...
   % 'MarkerEdgeColor','b',...
    %'MarkerFaceColor',[0.5,0.5,0.5])
A_escalado = scale(A, 1.2, [centro_A_x, centro_A_y]);
%plot(A_escalado)

rca=sqrt((centro_A_x-a3(1)).^2+(centro_A_y-a3(2)).^2);

xf=0:0.2:15;
yf=0:0.2:15;
[X Y]=meshgrid(xf,yf);
goal=[13,6];
obs1=[centro_A_x,centro_A_y];
obs2=[centro_B_x,centro_B_y];
obs3=[centro_C_x,centro_C_y];
kg=50;
ko=25;
rg=sqrt((goal(1)-X).^2+(goal(2)-Y).^2);
vg=kg*rg; %Campo potencial para o objetivo

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
        obs=[a1;a2;a3;b1;b2;b3;b4;b5;c1;c2;c3;obsa1;obsa2;obsa3;obsb1;obsb2;obsb3;obsb4;obsb5;obsc1;obsc2;obsc3];
        
        
        vo=zeros(size(X));

        for i=1:length(obs)
            ro=sqrt((obs(i,1)-X).^2+(obs(i,2)-Y).^2);
            vo=vo+ ko./ro;
        end

%{
ro1=sqrt((obs1(1)-X).^2+(obs1(2)-Y).^2); %Campo potencial para o primeiro obstáculo
vo1=ko./ro1;

ro2=sqrt((obs2(1)-X).^2+(obs2(2)-Y).^2); %Campo potencial para o primeiro obstáculo
vo2=ko./ro2;

ro3=sqrt((obs3(1)-X).^2+(obs3(2)-Y).^2); %Campo potencial para o primeiro obstáculo
vo3=ko./ro3;
%}
mesh(xf,yf,vg+vo)
%axis([0 15 0 15 0 400])


