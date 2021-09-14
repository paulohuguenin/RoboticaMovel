vdims = vehicleDimensions;
egoWheelbase = vdims.Wheelbase;
distToCenter = 0.5*egoWheelbase;
egoInitialPose = [7,3.1,0];
egoTargetPose = [-distToCenter,0,0];

Tv = 0.1;
helperSLVisualizeParking(egoInitialPose,0);

xlim = [-10 10];   
ylim = [-2 6];     
yawlim = [-3.1416 3.1416]; 
bounds = [xlim;ylim;yawlim];
stateSpace = stateSpaceReedsShepp(bounds);
stateSpace.MinTurningRadius = 7;

stateValidator = parkingStateValidator(stateSpace);

planner = plannerRRTStar(stateSpace,stateValidator);
planner.MaxConnectionDistance = 4;
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 2000;

rng(9, 'twister');
[pathObj,solnInfo] = plan(planner,egoInitialPose,egoTargetPose);

f = findobj('Name','Automated Parallel Parking');
ax = gca(f);
hold(ax, 'on');
plot(ax,solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'y.-'); % tree expansion

p = 100;
pathObj.interpolate(p+1);
xRef = pathObj.States;

plot(ax,xRef(:,1), xRef(:,2),'b-','LineWidth',2)

mpcverbosity('off');

nlobjTracking = nlmpc(3,3,2);

Ts = 0.1;
pTracking = 10;
nlobjTracking.Ts = Ts;
nlobjTracking.PredictionHorizon = pTracking;
nlobjTracking.ControlHorizon = pTracking;

nlobjTracking.MV(1).Min = -2;
nlobjTracking.MV(1).Max = 2;
nlobjTracking.MV(2).Min = -pi/6;
nlobjTracking.MV(2).Max = pi/6;

nlobjTracking.Weights.OutputVariables = [1,1,3]; 
nlobjTracking.Weights.ManipulatedVariablesRate = [0.1,0.2];

nlobjTracking.Model.StateFcn = "parkingVehicleStateFcnRRT";
nlobjTracking.Jacobian.StateFcn = "parkingVehicleStateJacobianFcnRRT";


nlobjTracking.Optimization.CustomEqConFcn = "parkingTerminalConFcn";

validateFcns(nlobjTracking,randn(3,1),randn(2,1));

x = egoInitialPose';

u = [0;0];

[coredata,onlinedata] = getCodeGenerationData(nlobjTracking,x,u);

mexfcn = buildMEX(nlobjTracking,'parkingRRTMex',coredata,onlinedata);

xTrackHistory = x;
uTrackHistory = u;
mv = u;
Duration = 14;
Tsteps = Duration/Ts;
Xref = [xRef(2:p+1,:);repmat(xRef(end,:),Tsteps-p,1)];

for ct = 1:Tsteps
    % States
    xk = x;
    % Compute optimal control moves with MEX function
    onlinedata.ref = Xref(ct:min(ct+pTracking-1,Tsteps),:);
    [mv,onlinedata,info] = mexfcn(xk,mv,onlinedata);
    % Implement first optimal control move and update plant states.
    ODEFUN = @(t,xk) parkingVehicleStateFcnRRT(xk,mv);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xk);
    x = YOUT(end,:)';
    % Save plant states for display.
    xTrackHistory = [xTrackHistory x]; %#ok<*AGROW>
    uTrackHistory = [uTrackHistory mv];
end

plotAndAnimateParkingRRT(p,xRef,xTrackHistory,uTrackHistory);