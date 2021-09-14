clc;
clear all;
tic


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%     MAPA     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i1 = [1;3];
i2 = [1;6];

f1 = [9;8];
f2 = [9;6];
f3 = [9;3];

xlim = 10;
ylim = 10;
res = 100;

map = binaryOccupancyMap(xlim, ylim, res);

new_m = getMatrixOccLine(map, 2, 3, 6, -6);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 3, 4, 3, 6);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 2, 4, 6, 0);
setOccupancy(map, [0 0], new_m);

mapmatrix = occupancyMatrix(map);
poly = fill_map(map, [2 8], [1 6]);
mapmatrix(2*res:8*res, 1*res:6*res) = poly;
setOccupancy(map, [0 0], mapmatrix)

new_m = getMatrixOccLine(map, 5, 5, 6, 2*res);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 5, 7, 8, 0);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 7, 8, 8, -2);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 7, 8, 5, 4);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 5, 7, 6, -1);
setOccupancy(map, [0 0], new_m);

mapmatrix = occupancyMatrix(map);
poly = fill_map(map, [1.8 5.2], [4.8 8.2]);
mapmatrix(1.8*res:5.2*res, 4.8*res:8.2*res) = poly;
setOccupancy(map, [0 0], mapmatrix)

new_m = getMatrixOccLine(map, 5, 8, 2, 0);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 5, 6, 2, 4);
setOccupancy(map, [0 0], new_m);
new_m = getMatrixOccLine(map, 6, 8, 4, -2);
setOccupancy(map, [0 0], new_m);

mapmatrix = occupancyMatrix(map);
poly = fill_map(map, [5.9 8.1], [4.9 8.1]);
mapmatrix(5.9*res:8.1*res, 4.9*res:8.1*res) = poly;
setOccupancy(map, [0 0], mapmatrix)

%inflate(map, 0.2)
%figure,
%show(map)


prmSimple = mobileRobotPRM(map,50);
prmSimple.ConnectionDistance = 20;
%show(prmSimple)
%show(map)
for j=[f3]
for i=[i1]
startLocation = i';
endLocation = j';
%rng(rngState);
%prmSimple.ConnectionDistance = 6;
path = findpath(prmSimple,startLocation,endLocation);
%{
for i=1:length(path)-1
ipPath=genObsPoly(path(i,:),path(i+1,:),4);
if i==1
    auxnp=[path(i,:);ipPath];
    np=auxnp;
else
    np=vertcat(np,[path(i,:);ipPath]);
end
    
end
path=np;
%}
figure,
%show(prmSimple)

show(map)
%show(prmSimple)
%figure,plot(path)
hold on

sampleTime = 0.2;               % Sample time [s]
tVec = 0:sampleTime:50; 

bicycle = bicycleKinematics("VehicleInputs","VehicleSpeedHeadingRate","MaxSteeringAngle",pi/4,"WheelBase",0.2);
initPose = [startLocation'; 0]; % Initial pose (x y theta)
controller2 = controllerPurePursuit("Waypoints",path,"DesiredLinearVelocity",0.5,"MaxAngularVelocity",pi,"LookaheadDistance",0.4);

goalPoints = vertcat(path(end,:)',0);
goalRadius = 1;

[tBicycle,bicyclePose] = ode45(@(t,y)derivative(bicycle,y,exMobileRobotController(controller2,y,goalPoints,goalRadius)),tVec,initPose);

bicycleTranslations = [bicyclePose(:,1:2) zeros(length(bicyclePose),1)];
bicycleRot = axang2quat([repmat([0 0 1],length(bicyclePose),1) bicyclePose(:,3)]);
%hold on
plotTransforms(bicycleTranslations(1:15:end,:),bicycleRot(1:15:end,:),'MeshFilePath','groundvehicle.stl',"MeshColor","b","FrameSize",0.4);
plot(bicyclePose(:,1),bicyclePose(:,2),'LineWidth',1,'DisplayName','Trajetória Controlada');
plot(startLocation(1,1),startLocation(1,2),'s', 'MarkerFaceColor','b','MarkerSize',10,'HandleVisibility','off');
hold on
plot(endLocation(1,1),endLocation(1,2),'s','MarkerFaceColor','r','MarkerSize',10,'HandleVisibility','off');
plot(path(:,1),path(:,2),...%{'Color',[rand(), rand() ,rand()],
    'LineWidth',1,'DisplayName','Trajetória PRM' )
title('Trajetória por PRM');
xlabel('X');
ylabel('Y');
legend('Location','northwest');
hold oncle
view(0,90)

end
end
%figure,plot(tBicycle,bicyclePose)
toc

function mapmatrix = getMatrixOccLine(map, startx, endx, starty, yxratio)

    xlim = map.XWorldLimits;
    ylim = map.YWorldLimits;
    resolution = map.Resolution;
    
    startx = startx * resolution;
    endx = endx * resolution;
    starty = starty * resolution;
    
    if yxratio < 0
        stepy = -1;
    else
        stepy = 1;
    end
    
    if endx < startx
        stepx = -1;
    else
        stepx = 1;
    end
    
    mapmatrix = occupancyMatrix(map);
    y = ylim(2)*resolution - starty;
    
    for x = startx:stepx:endx    
        mapmatrix(y, x) = 1;        
        if mod(x, 2) == 0
            for i = 0:stepy:yxratio
                mapmatrix(y-i, x) = 1;
            end
            y = y - yxratio;
        end
    end
end

function poly = fill_map(map, xlims, ylims)
    
    resolution = map.Resolution;
    
    xstart = resolution*xlims(1);
    xend = resolution*xlims(2);
    ystart = resolution*ylims(1);
    yend = resolution*ylims(2);
    
    mapmatrix = occupancyMatrix(map);
    poly = mapmatrix(xstart:xend, ystart:yend);
    
    for row = 1:size(poly, 1) 
        found_first_ones = 0;
        found_second_ones = 0;
        start_x_fill = 0;
        end_x_fill = 0;
        for col = 1:size(poly, 2)
            if found_first_ones == 0 && poly(row, col) == 1
                found_first_ones = 1;
            end
            if found_first_ones == 1 && poly(row, col) == 1
                continue;
            end
            if found_first_ones == 1 && poly(row, col) == 0
                start_x_fill = col;
                found_first_ones = 2; % para de lidar com 1 grupo
            end
            if found_first_ones == 2 && poly(row, col) == 0
                continue;
            end
            if found_first_ones == 2 && poly(row, col) == 1
                found_second_ones = 1;
                end_x_fill = col - 1;
            end
        end
        if start_x_fill ~= 0 && end_x_fill ~= 0
            poly(row, start_x_fill:end_x_fill) = 1;
        end
    end
end

    