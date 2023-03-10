%% Solve a basic dynamic optimization problem
% Clear command window
clc;

% Clear variables
clear all; %#ok

% Close figures
close all;

% Remove added paths
restoredefaultpath;

%% Formatting
% Font size
fs = 12;

% Line width
lw = 3;

% Marker size
ms = 12;

% Set default font size
set(groot, 'DefaultAxesFontSize',   fs);

% Set default line widths
set(groot, 'DefaultLineLineWidth',  lw);
set(groot, 'DefaultStairLineWidth', lw);
set(groot, 'DefaultStemLineWidth',  lw);

% Set default marker size
set(groot, 'DefaultLineMarkerSize', ms);

% Set default figure size
set(groot, 'DefaultFigureUnits', 'Inches', 'DefaultFigurePosition', [1, 1, 8, 6]);

% Set renderer
% (Otherwise, eps figures can become pixelated in Latex.)
% (Save as pdf if you have shaded areas in your figure, e.g., because
% you're using fill.)
set(groot, 'DefaultFigureRenderer', 'Painters');

%% Initialize
% Reference
yref = 0.5;

% Right-hand side function
fun = @(t, x, u) -x + u;

% Measurement function
output = @(t, x, u) x;

% Stage cost
stageCost = @(t, x, u, y) (y - yref).^2;

%% Simulate
% Initial condition
x0 = 0;

% Manipulated input
u = 1;

% Time span
tspan = linspace(0, 10, 101);

% Options
odeopts = odeset();

% Simulate
[t, x] = ode45(fun, tspan, x0, odeopts, ...
    u);

% Create figure
figure(1);

% Visualize simulation
plot(t, x);

% Add more plots
hold on;

% Plot inputs
plot(t, repmat(u, size(t)));

% Stop adding plots
hold off;

% Axis limits
ylim([-0.1, 1.1]);

% Title
title('Simulation');

%% Optimize
% Inputs for fmincon
A       = [];
B       = [];
Aeq     = [];
Beq     = [];
nonlcon = [];

ub = 1;
lb = 0;

% Options for fmincon
optopts = optimset(    ...
    'Display', 'iter', ...
    'TolFun', 1e-8);

% Initial guess of optimal control
u0 = 0.1;

% Use fmincon to solve the numerical optimization problem
uopt = fmincon(@evaluateObjectiveFunction, u0, A, B, Aeq, Beq, lb, ub, nonlcon, optopts, ...
    x0, tspan, fun, output, stageCost, odeopts);

%% Simulate
% Simulate
[topt, xopt] = ode45(fun, tspan, x0, odeopts, ...
    uopt);

% Create figure
figure(2);

% Plot setpoint
plot(t, repmat(yref, size(t)), '--k');

% Reset color ordering
set(gca, 'ColorOrderIndex', 1);

% Add more plots
hold on;

% Plot inputs
plot(t, repmat(uopt, size(t)));

% Visualize simulation
plot(topt, xopt);

% Stop adding plots
hold off;

% Axis limits
ylim([-0.1, 1.1]);

% Title
title('Optimization');

function J = evaluateObjectiveFunction(u, x0, tspan, fun, outp, stageCost, odeopts)

% Simulate
[t, x] = ode45(fun, tspan, x0, odeopts, ...
    u);

% Evaluate outputs
y = outp(t, x, u);

% Evaluate stage cost
Phi = stageCost(t, x, u, y);

% Evaluate objective function
J = sum(Phi(2:end).*diff(t(:)));
end