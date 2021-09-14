function J = goalfunction(x,xgoal,w2) 


%r = 1.5 %
%r = 2.25

%if theta> pi
 %   theta=theta-2*pi;
%end

%d=sqrt((x(1,1)-xgoal(1,1))^2 + (x(2,1)-xgoal(2,1))^2);
 %   custo= sqrt((d)^2 + 0.5*(theta-pi/2)^2);
%custo=d;
d = x - xgoal;
d = sum(d.^2); % ???????

J = w2*2*(exp(-0.5*d)); % ????
J = J+w2*10*(exp(-0.01*d));

%else
%    J = w2*2*(exp(-0.5*1));
 %   J = J+w2*10*(exp(-0.01*1));
%end
