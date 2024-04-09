clc; clear;
clear all;
close all;

%% Setup parameters
addpath('./mip', './rrt', './dynamics')
xr          = [0;0];
x0          = [9;9];
traj        = x0;
lineWidth   = 3;
FontSize    = 25;
plant       = ominidirectionalRobot(x0);
network     = nndm(plant);
rgRRT       = reachabilityGuidedRRT(x0, xr, plant, network);
reachSet    = {};

%% Initialze figure
fig     = figure(1);
set(gcf,'color','w');
set(gcf,'Position',[0 0 800 800]);
plot(zonotope(plant.X), [1,2], 'k', 'lineWidth', 0.7*lineWidth); hold on;
hi  = plot(traj(1), traj(2), 'rx', 'markerSize', 12, 'lineWidth', lineWidth); hold on
hf  = plot(xr(1), xr(2), 'ro', 'markerSize', 12, 'lineWidth', lineWidth);
for i = 1:length(plant.X_us)
    hXu     = plot(zonotope(plant.X_us{i}), [1,2], 'k', 'lineWidth', 0.7*lineWidth, 'FaceColor', [227,114,34]/255, 'Filled', true);
end
xlim([-1, 11]); ylim([-1, 11]); grid on; pbaspect([1 1 1]);
xlabel('X', 'FontSize', FontSize); ylabel('Y', 'FontSize', FontSize);
set(gca, 'FontSize', FontSize);

%% Reachability Guided RRT
new_reference_trajectory    = false;
if new_reference_trajectory % generate a new reference trajectory
    rgRRT.exploreStateSpace(true);
else % use pre-generated ones
    load("rgRRT.mat", 'rgRRT')
    rgRRT.exploreStateSpace(true);
end
shortestRefTrajectory   = rgRRT.shortestRefTrajectory;
exportgraphics(fig, './rrtPlanedPath.png', 'Resolution',300)

%% Main Safe OCP
xr  = shortestRefTrajectory(:,1);
shortestRefTrajectory(:,1)  = [];
is_termination              = false;

computation_time    = [];

while ~is_termination            
    % -------------------------------------------------------------
    % Solve OptCtrl problem for u_til
    % -------------------------------------------------------------
    opt     = safeOptCtrl(xr, plant, network);
    tic;
    result  = opt.solveSafeOptCtrlProblem();
    time_elapsed        = toc;
    computation_time    = [computation_time, time_elapsed];
    % -------------------------------------------------------------
    % Visualization
    % -------------------------------------------------------------
    if exist('hTraj','var')
        delete(hTraj);        
    end
    % -------------------------------------------------------------
    % Emulate ground true dynamics
    % -------------------------------------------------------------
    if result.problem ~= 0
        disp('no solution');
        continue
    else
        plant.updateDynamicSystem(value(opt.u_tild));
    end
    % -------------------------------------------------------------
    % Visualization
    % -------------------------------------------------------------
    xkp1                = plant.xk;
    traj                = [traj, xkp1];
    reachSet{end+1}     = zonotope(interval(value(opt.x_kp1_l), value(opt.x_kp1_u)));
    
    hSet    = plot(reachSet{end}, [1,2], 'g', 'lineWidth', lineWidth);
    hTraj   = plot(traj(1,:), traj(2,:), '*', 'color', 'blue', 'markerSize', 10, 'lineWidth', 0.5*lineWidth);
    if isempty(plant.X_us)        
        legend([hi, hf, hTraj, hSet],{'start $x_{0}$ ', 'goal $x^{g}$ ',...
            'actual state $x_{k+1}$ ', 'reachable set $[\underline{x}_{k+1},\overline{x}_{k+1}]$ '}, 'Interpreter',...
            'latex', 'FontSize', 15, 'NumColumns', 2, 'Location', 'northwest');
    else
        legend([hi, hf, hTraj, hSet, hXu],{'start $x_{0}$ ', 'goal $x_{g}$ ',...
            'actual state $x_{k+1}$ ', 'reachable set $[\underline{x}_{k+1},\overline{x}_{k+1}]$ ', 'obstacles $X_{u}^{(i)}$ '},...
            'Interpreter', 'latex', 'FontSize', 15, 'NumColumns', 2, 'Location', 'northwest');
    end
    % -------------------------------------------------------------
    % Update reference point and check the termination condition
    % -------------------------------------------------------------
    if ~in(reachSet{end}, xkp1)
        warning('fail to contain');
    end
    if in(reachSet{end}, xr) && ~isempty(shortestRefTrajectory)
        xr  = shortestRefTrajectory(:,1);
        shortestRefTrajectory(:,1)  = [];
    end
    is_termination  = (isempty(shortestRefTrajectory) && in(reachSet{end}, xr));
    pause(0.01)
end
exportgraphics(fig, './safeTracking.png', 'Resolution',300)