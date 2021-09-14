
function [t,x]=potentialFieldCar()

clc

close all

time=[0 300];
x0=[0;0;0];
[t, x]=ode23(@car,time,x0);
figure,
plot(x(:,1),x(:,2),'r');
hold on
plot(goal(1),goal(2),'o',obs1(1),obs1(2),'x',obs2(1),obs2(2),'x');
axis([0 12 0 12])

    function dx=car(t,x)
        
        goal=[10;10];
        obs1=[3;3];
        obs2=[9;9];
        kg=30;
        ko=30;
        
        rg=sqrt((goal(1)-x(1))^2+(goal(2)-x(2))^2);
        fgx=kg*(goal(1)-x(1))/rg;
        fgy=kg*(goal(2)-x(2))/rg;
        %vg=kg*rg; %Campo potencial para o objetivo

        ro1=sqrt((obs1(1)-x(1))^2+(obs1(2)-x(2))^2); %Campo potencial para o primeiro obstáculo
        fo1x=-ko*(obs1(1)-x(1))/ro1^3;
        fo1y=-ko*(obs1(2)-x(2))/ro1^3;
        %vo1=ko./ro1;

        ro2=sqrt((obs2(1)-x(1))^2+(obs2(2)-x(2))^2); %Campo potencial para o primeiro obstáculo
        fo2x=-ko*(obs2(1)-x(1))/ro2^3;
        fo2y=-ko*(obs2(2)-x(2))/ro2^3;
        %vo2=ko./ro2;
        
        fx=(fgx+fo1x+fo2x);
        fy=(fgy+fo1y+fo2y);
        
        alpha=atan(fy/fx);
        
        v=1;
        L=2;
        
        if rg<0.05
            v=0;
        end
        
        K=2;
        ph=K*(alpha-x(3));
        dx=[v*cos(x(3));v*sin(x(3));];
    end
end
