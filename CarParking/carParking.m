clc;
clear;
close all;
l = 26; %wheelbase - mm
w = 15; %tread - mm
v = 5;  %vehicle speed - m/s
v_d = 5;
delta_t =0.8; %Sampling interval - second

x=20; %The strating X coordinate of the vehicle
x_d=-9.999;
y=45; %The strating Y coordinate of the vehicle
y_d=45; 
theta=0; %Automobile body initial inclination
theta_d=0;
phi=0; %max 45 degree

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Go stright until find the parking place%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_x=v*cos(theta)*cos(phi)*(delta_t);
delta_x_d=v_d*cos(theta)*cos(phi)*(delta_t);
delta_y=v*sin(theta)*cos(phi)*(delta_t);
delta_y_d=v_d*sin(theta)*cos(phi)*(delta_t);
delta_theta=(v*sin(phi)/l)*(delta_t);
delta_theta_d=(v_d*sin(phi)/l)*(delta_t);
x_seq=zeros(1,1);
y_seq=zeros(1,1);
theta_seq=zeros(1,1);
phi_seq=zeros(1,1);
figure; %draw the empty picture 
pause(1);
for i = 1:28
    pause(0.3);
x=x+delta_x; %calculate the newest x coordinate
x_d=x_d+delta_x_d;
y=y+delta_y; %calculate the newest y coordinate
y_d=y_d+delta_y_d;
theta=theta+delta_theta; %calculate the new automobile inclination 
theta_d=theta_d+delta_theta_d;
  x_seq(i) = x; %存储 x 坐标到 x 序列store x coordinate to x sequence
  y_seq(i) = y; %store y coordinate to y sequence
  theta_seq(i) = theta; %store theta coordinate to theta sequence
  phi_seq(i) = phi; %store phi coordinate to phi theta sequence
      %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
x0 = x + w/2*sin(theta); %x coordinate in the left front corner of the car
x0_d = x_d + w/2*sin(theta_d);
y0 = y - w/2*cos(theta); %y coordinate in the left front corner of the car
y0_d = y_d - w/2*cos(theta_d);

x1 = x - w/2*sin(theta); %y coordinate in the right front corner of the car
x1_d = x_d - w/2*sin(theta_d);
y1 = y + w/2*cos(theta); %y coordinate in the right front corner of the car
y1_d = y_d + w/2*cos(theta_d);

p = x - l*cos(theta); 
p_d = x_d - 1*cos(theta_d);
q = y - l*sin(theta);
q_d = y_d - 1*sin(theta_d);

x2 = p + w/2*sin(theta); %x coordinate in the left rear corner of the car
x2_d = p_d + w/2*sin(theta_d);
y2 = q - w/2*cos(theta); %y coordinate in the left rear corner of the car
y2_d = q_d - w/2*sin(theta_d);

x3 = p - w/2*sin(theta); %x coordinate in the right rear corner of the car
x3_d = p_d - w/2*sin(theta_d);
y3 = q + w/2*cos(theta); %y coordinate in the right rear corner of the car
y3_d = q_d + w/2*cos(theta_d);

%Draw the  center of the rear axle of an automobile
plot (x, y, 'rs');
plot (x_d, y_d, 'w');
axis([-20 160 0 100]);
xlabel('x - cm');
ylabel('y - cm');
title('Parallel Parking ');
hold on
grid on;

%Draw a diagram of the parking space
h1 = line([-20 40], [60 60]);
h2 = line([40 40], [60 85]);
h3 = line([40 105], [85 85]);
h4 = line([105 105], [60 85]);
h5 = line([105 160], [60 60]);
h6 = line([-20 160], [20 20]);
     set(h1, 'linewidth', 5, 'color', 'b');
     set(h2, 'linewidth', 5, 'color', 'b');
     set(h3, 'linewidth', 5, 'color', 'b');
     set(h4, 'linewidth', 5, 'color', 'b');
     set(h5, 'linewidth', 5, 'color', 'b');
     set(h6, 'linewidth', 5, 'color', 'b');
     
     %Draw the outline of the automobile body
     l0 = line([x0 x1], [y0 y1]);
     l1 = line([x1 x3], [y1 y3]);
     l2 = line([x2 x3], [y2 y3]);
     l3 = line([x0 x2], [y0 y2]);

     set(l0, 'linewidth', 4, 'color', 'm');
     set(l1, 'linewidth', 2, 'color', 'b');
     set(l2, 'linewidth', 4, 'color', 'g');
     set(l3, 'linewidth', 2, 'color', 'b');
      
     l4 = line([x0_d x1_d], [y0_d y1_d]);
     l5 = line([x1_d x3_d], [y1_d y3_d]);
     l6 = line([x2_d x3_d], [y2_d y3_d]);
     l7 = line([x0_d x2_d], [y0_d y2_d]);
      
     set(l4, 'linewidth', 4, 'color', 'w');
     set(l5, 'linewidth', 2, 'color', 'w');
     set(l6, 'linewidth', 4, 'color', 'w');
     set(l7, 'linewidth', 2, 'color', 'w');
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%A little backward%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=132; %汽车起点的 x 坐标
y=45; %汽车起点的 y 坐标
theta=0; %汽车车身起始倾角
phi=0; %max 45 degree

delta_x=v*cos(phi)*cos(theta)*(delta_t);
delta_y=v*sin(theta)*cos(phi)*(delta_t);
delta_theta=[v*sin(phi)/l]*(delta_t);

for i = 1:4
     pause(0.3);
x=x-delta_x; 
y=y-delta_y; 
theta=theta+delta_theta;
  x_seq(i) = x; 
  y_seq(i) = y; 
  theta_seq(i) = theta; 
  phi_seq(i) = phi; 
      %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
x0 = x + w/2*sin(theta); 
y0 = y - w/2*cos(theta); 

x1 = x - w/2*sin(theta); 
y1 = y + w/2*cos(theta); 

p = x - l*cos(theta);  
q = y - l*sin(theta);

x2 = p + w/2*sin(theta); 
y2 = q - w/2*cos(theta); 

x3 = p - w/2*sin(theta); 
y3 = q + w/2*cos(theta); 

plot (x, y, 'rs');
axis([-20 160 0 100]);
xlabel('x - cm');
ylabel('y - cm');
title('Parallel Parking ');
hold on
grid on;

     %画车身轮廓
     l0 = line([x0 x1], [y0 y1]);
     l1 = line([x1 x3], [y1 y3]);
     l2 = line([x2 x3], [y2 y3]);
     l3 = line([x0 x2], [y0 y2]);

     set(l0, 'linewidth', 4, 'color', 'm');
     set(l1, 'linewidth', 2, 'color', 'b');
     set(l2, 'linewidth', 4, 'color', 'g');
     set(l3, 'linewidth', 2, 'color', 'b');
     
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%The first sterring%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=116; 
y=45; 
theta=0; 
phi=10.04; 

delta_x=v*cos(phi)*cos(theta)*(delta_t);
delta_y=v*sin(theta)*cos(phi)*(delta_t);
delta_theta=[v*sin(phi)/l]*(delta_t);

for i = 1:9
     pause(0.3);
x=x+delta_x; 
y=y+delta_y; 
theta=theta+delta_theta; 
  x_seq(i) = x; 
  y_seq(i) = y; 
  theta_seq(i) = theta; 
  phi_seq(i) = phi; 
      %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
x0 = x + w/2*sin(theta); 
y0 = y - w/2*cos(theta); 

x1 = x - w/2*sin(theta); 
y1 = y + w/2*cos(theta); 

p = x - l*cos(theta);  
q = y - l*sin(theta);

x2 = p + w/2*sin(theta); 
y2 = q - w/2*cos(theta); 

x3 = p - w/2*sin(theta); 
y3 = q + w/2*cos(theta); 

plot (x, y, 'rs');
axis([-20 160 0 100]);
xlabel('x - cm');
ylabel('y - cm');
title('Parallel Parking ');
hold on
grid on;

     l0 = line([x0 x1], [y0 y1]);
     l1 = line([x1 x3], [y1 y3]);
     l2 = line([x2 x3], [y2 y3]);
     l3 = line([x0 x2], [y0 y2]);

     set(l0, 'linewidth', 4, 'color', 'm');
     set(l1, 'linewidth', 2, 'color', 'b');
     set(l2, 'linewidth', 4, 'color', 'g');
     set(l3, 'linewidth', 2, 'color', 'b');   
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%opposite direction parking%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=68.47; 
y=63.64; 
theta=-0.7999; 
phi=39.9; 

delta_x=v*cos(phi)*cos(theta)*(delta_t);
delta_y=v*sin(theta)*cos(phi)*(delta_t);
delta_theta=[v*sin(phi)/l]*(delta_t);

for i = 1:6
     pause(0.3);
x=x+delta_x; 
y=y+delta_y; 
theta=theta+delta_theta; 
  x_seq(i) = x; 
  y_seq(i) = y; 
  theta_seq(i) = theta; 
  phi_seq(i) = phi; 
      %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
x0 = x + w/2*sin(theta); 
y0 = y - w/2*cos(theta); 

x1 = x - w/2*sin(theta); 
y1 = y + w/2*cos(theta); 


p = x + l*cos(theta);  
q = y + l*sin(theta);

x2 = p + w/2*sin(theta); 
y2 = q - w/2*cos(theta); 

x3 = p - w/2*sin(theta); 
y3 = q + w/2*cos(theta);



plot (x, y, 'rs');
axis([-20 160 0 100]);
xlabel('x - cm');
ylabel('y - cm');
title('Parallel Parking ');
hold on
grid on;

     l0 = line([x0 x1], [y0 y1]);
     l1 = line([x1 x3], [y1 y3]);
     l2 = line([x2 x3], [y2 y3]);
     l3 = line([x0 x2], [y0 y2]);

     set(l0, 'linewidth', 4, 'color', 'm');
     set(l1, 'linewidth', 2, 'color', 'b');
     set(l2, 'linewidth', 4, 'color', 'g');
     set(l3, 'linewidth', 2, 'color', 'b');
     
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%slight move the body of the car%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=58.615; 
y=73.78; 
theta=31.3;
phi=5; 
pause(0.1);
x=x+delta_x; 
y=y+delta_y; 
theta=theta+delta_theta; 
x0 = x + w/2*sin(theta); 
y0 = y - w/2*cos(theta); 
x1 = x - w/2*sin(theta); 
y1 = y + w/2*cos(theta); 

p = x + l*cos(theta);  
q = y + l*sin(theta);

x2 = p + w/2*sin(theta); 
y2 = q - w/2*cos(theta); 

x3 = p - w/2*sin(theta); 
y3 = q + w/2*cos(theta); 

l0 = line([x0 x1], [y0 y1]);
l1 = line([x1 x3], [y1 y3]);
l2 = line([x2 x3], [y2 y3]);
l3 = line([x0 x2], [y0 y2]);

set(l0, 'linewidth', 4, 'color', 'k');
set(l1, 'linewidth', 2, 'color', 'k');
set(l2, 'linewidth', 4, 'color', 'k');
set(l3, 'linewidth', 2, 'color', 'k');