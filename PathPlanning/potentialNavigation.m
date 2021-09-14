function [t,x]=potentialNavigation()
clc

close all



a1 = [2, 6];
a2 = [4, 6];
a3 = [3, 3];

b1 = [5+2, 8+2];
b2 = [7+2, 8+2];
b3 = [8+2, 7+2];
b4 = [7+2, 5+2];
b5 = [5+2, 6+2];

c1 = [6+2, 4];
c2 = [8+2, 2];
c3 = [5+2, 2];



i1 = [2;4];
i2 = [2; 6];

f1 = [13; 9];
f2 = [13; 6];
f3 = [13; 3];

A = polyshape([a1; a2; a3]);
B = polyshape([b1; b2; b3; b4; b5]);
C = polyshape([c1; c2; c3]);

[centro_A_x, centro_A_y] = centroid(A);
[centro_B_x, centro_B_y] = centroid(B);
[centro_C_x, centro_C_y] = centroid(C);

time=[0 300];
x0=[0;4;0];
[t x]=ode23(@car,time,x0);
figure,
plot(x(:,1),x(:,2),'r');
hold on
plot([A, B, C]);
plot(goal(1),goal(2),'o');
for i=1:length(obs)
    plot(obs(i,1),obs(i,2),'x');
end
axis([0 30 0 30])


    function dx=car(t,x)
        
        goal=[11;5];
        %obs1=[3;3];
        %obs2=[9;9];
        %obs1=[centro_A_x,centro_A_y];
        %obs2=[centro_B_x,centro_B_y];
        %obs3=[centro_C_x,centro_C_y];
        n=5;
        
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
        
        kg=30;
        ko=30;
        
        rg=sqrt((goal(1)-x(1))^2+(goal(2)-x(2))^2);
        fg=[kg*(goal(1)-x(1))/rg;kg*(goal(2)-x(2))/rg];
        %fgy=kg*(goal(2)-x(2))/rg;
        %vg=kg*rg; %Campo potencial para o objetivo

        for i=1:length(obs)
            ro(i)=sqrt((obs(i,1)-x(1))^2+(obs(i,2)-x(2))^2);
            fox(i)=-ko*(obs(i,1)-x(1))/ro(i)^3;
            foy(i)=-ko*(obs(i,2)-x(2))/ro(i)^3;
        end
        
        %{
        ro1=sqrt((obs1(1)-x(1))^2+(obs1(2)-x(2))^2); %Campo potencial para o primeiro obstáculo
        fo1x=-ko*(obs1(1)-x(1))/ro1^3;
        fo1y=-ko*(obs1(2)-x(2))/ro1^3;
        %vo1=ko./ro1;

        ro2=sqrt((obs2(1)-x(1))^2+(obs2(2)-x(2))^2); %Campo potencial para o primeiro obstáculo
        fo2x=-ko*(obs2(1)-x(1))/ro2^3;
        fo2y=-ko*(obs2(2)-x(2))/ro2^3;
        
        ro3=sqrt((obs3(1)-x(1))^2+(obs3(2)-x(2))^2); %Campo potencial para o primeiro obstáculo
        fo3x=-ko*(obs3(1)-x(1))/ro3^3;
        fo3y=-ko*(obs3(2)-x(2))/ro3^3;
        %vo2=ko./ro2;
        %}
        
        %fx=(fgx+fo1x+fo2x+fo3x);
        %fy=(fgy+fo1y+fo2y+fo3y);
        
        ftx=fg(1)+sum(fox(:,1));
        fty=fg(2)+sum(foy(:,2));
        
        alpha=atan(fty/ftx);
        
        v=1;
        L=2;
        
        if rg<0.05
            v=0;
        end
        
        K=2;
        ph=K*(alpha-x(3));
        dx=[v*cos(ph)*cos(x(3));v*cos(ph)*sin(x(3));v*sin(ph)/L];
    end
end
