%% ...SET YOUR CONTROL VARIABLES HERE...

% -- Some info --
% The state vector is a = [Cd, Cl, dCd_dt, dCl_dt]
% The input vector is b = [b_1, b_2, b_3]
% where: 
% 1 - Front  cylinder 
% 2 - Top    cylinder 
% 3 - Bottom cylinder 


%% Parameters for MPC
Ts            = 0.5;                     % Time between MPC optimization (must be multiple of 0.05) [c.u.] (Default: 0.5)
dt            = 0.1;                     % Time interval for plant model integration (must be a divisor of Ts) [c.u.] (Default: 0.1)
StartControl  = 50;                      % Time when MPC begins (must be multiple of Ts) [c.u.] (Default: 50)
Duration      = 200;                     % Total simulation duration (must be multiple of Ts) [c.u.] (Default: 200)
AddNoise      = false;                   % Switch to "true" to consider measurement noise during MPC implementation
NoiseLevel    = 0;                       % Level of noise as a percentage of full-scale unforced Cd and Cl
UseLPR        = false;                   % Apply LPR to online state measurements during MPC
MaxIterMPCopt = 500;                     % Max iterations for MPC optimization (Default: 500) 
Feedcstartype = 0;                       % Parameter for reference trajectory in MPC cost functional (0: considers history of cstar, other: only first component constant in the prediction/control window)


% MPCPlotVerbose can take on three different values:
% If MPCPlotVerbose = 0, Plot results only at the end
% If MPCPlotVerbose = 1, Plot results during simulation
% If MPCPlotVerbose = 2, Additional plot of the fluidic pinball cylinders
% with actuation values is provided
% (Default: 2)

MPCPlotVerbose = 2;


%% Constraints of MPC (Plant model is trained with |b|_max = 1)
LB        = -1;                           % Lower bound of the control input [D/(c.u.)] (Default: -1)
UB        = 1;                            % Upper bound of the control input [D/(c.u.)] (Default: 1)
Alpha_max = 8;                            % Max angular acceleration of the cylinders [D/(c.u.)^2] (Default: 1)


%% Tuning parameters
PerformTuningParams = false;              % Use Bayesian optimization for hyperparameter tuning
IncludeNtuning      = false;              % Include prediction/control window length in tuning
UseLPRtuning        = false;              % Use LPR during control tuning

% Extended case:
% eta = [Q1, Q2, Rdu1, Rdu2, Rdu3, Ru1, Ru2, Ru3]
% ExtendedSym:
% eta = [Q1, Q2, Rd1, Rdu23, Ru1, Ru23]
% Reduced
% eta = [Q1, Q2, Rdu, Ru]
% If IncludeNtuning is "true" also N is included

% In case of using BO for tuning parameters
TuningParams.TuningCase   = "ExtendedSym"; 
TuningParams.Duration     = 25;         % Duration of MPC simulations during parameter optimization (must be multiple of Ts) [c.u.] (Default: 30)
TuningParams.StartControl = 5;          % Time when MPC control starts during parameter optimization (must be multiple of Ts) [c.u.] (Default: 5)
TuningParams.DT           = 15;         % Time from end of each simulation to sample J_BO for parameter tuning

% Params Case Extended
TuningParams.Q1_lim    = [0 10];        % Search area for weight matrix Q = diag(Q1,Q2)
TuningParams.Q2_lim    = [0 10];        %
TuningParams.Ru1_lim   = [0 10];        % Search area for weight matrix Ru = diag(Ru1, Ru2, Ru3)
TuningParams.Ru23_lim  = [0 10];        % (limits for Ru2 and Ru3 are the same)
TuningParams.Rdu1_lim  = [0 10];        % Search area for weight matrix Ru = diag(Rdu1, Rud2, Rdu3) 
TuningParams.Rdu23_lim = [0 10];        % (limits for Rdu2 and Rdu3 are the same)
TuningParams.N_lim     = [1 4];         % Search area for prediction/control window

% Params Case Reduced
TuningParams.Ru_lim    = [0 10];
TuningParams.Rdu_lim   = [0 10];

% General Params for Bayesian Optimization
TuningParams.AcquisitionFunction      = 'expected-improvement';  % Acquisition function for NextPoint search during BO of parameters (Default: expected-improvement)
TuningParams.MaxObjectiveEvaluations  = 60;                      % Maximum number of BO iteration for parameter tuning
TuningParams.NumSeedingPt             = 10;                      % Number of seeding points to explore the search area before running BO of parameters



% Manual selection of parameters (in case PerformTuningParams set to false)
% The following parameters will be ignored in case BO is used!

% Best parameter for ControlCase 0 and no noise
N   =  4;                                  % Length of the prediction window (Default: 3 c.u.)
Q   =  [9.6924 7.1122];                    % Weight matrix for the state components errors
Rdu =  [9.6183 2.8331 2.8331];             % Penalization of Input Variability
Ru  =  [7.9812 0.026097 0.026097];         % Penalization of Input

% Best parameter for ControlCase 1 and no noise (step Cl)
% N   =  3.5;                                % Length of the prediction window (Default: 3 c.u.)
% Q   =  [9.3718 8.0339];                    % Weight matrix for the state components errors
% Rdu =  [5.2078 0.027429 0.027429];         % Penalization of Input Variability
% Ru  =  [0.076749 0.59942 0.59942];         % Penalization of Input

% Best parameter for ControlCase 2 and no noise (step Cl)
% N   =  3.5;                                % Length of the prediction window (Default: 3 c.u.)
% Q   =  [7.9464 8.0448];                    % Weight matrix for the state components errors
% Rdu =  [5.1807 0.060485 0.060485];         % Penalization of Input Variability
% Ru  =  [0.78682 0.62138 0.62138];          % Penalization of Input




%% Some checks of the variables selcted
run("Checks.m")


%% Control target for MPC
TimeControl = Duration - StartControl;   % Time of MPC


ControlCase = 0;
% Control target options
% 0:  Setpoint control for Cl/Cd (drag minimization)
% 1:  Reference trajectory for Cl (step function) and drag minimization
% 2:  Reference trajectory for Cl (sin function) and drag minimization
% 3:  CUSTOM - define your control reference

t_ref  = linspace(StartControl, Duration, 100);

% Some examples
switch ControlCase
    case 0
        Cd_ref = zeros(size(t_ref));
        Cl_ref = zeros(size(t_ref));

    case 1
        Cd_ref = zeros(size(t_ref));
        Cl_ref = interp1(StartControl + TimeControl/6*(0:5),  [-1.3 -0.5 2.5-1 1 0 -0.9], t_ref, "previous","extrap");

    case 2
        Cd_ref = zeros(size(t_ref));
        Cl_ref = sin(2*pi*(t_ref-StartControl)/TimeControl);

    otherwise
        Cd_ref = zeros(size(t_ref));
        Cl_ref = zeros(size(t_ref));
end




%% Control target for MPC simulation during parameter BO
% These data will be considered only in case of parameter BO
TuningParams.TimeControl = TuningParams.Duration - TuningParams.StartControl;   % Time of MPC during parameter BO
TuningParams.t_ref  = linspace(TuningParams.StartControl, TuningParams.Duration, 100);

% Some examples
switch ControlCase
    case 0
        TuningParams.Cd_ref = zeros(size(TuningParams.t_ref));
        TuningParams.Cl_ref = zeros(size(TuningParams.t_ref));

    case 1
        TuningParams.Cd_ref = zeros(size(TuningParams.t_ref));
        TuningParams.Cl_ref = interp1(TuningParams.StartControl + TuningParams.TimeControl/4*(0:3),  [0 -1 1 1.5], TuningParams.t_ref, "previous","extrap");

    case 2
        TuningParams.Cd_ref = zeros(size(TuningParams.t_ref));
        TuningParams.Cl_ref = sin(2*pi*(TuningParams.t_ref-TuningParams.StartControl)/TuningParams.TimeControl);

    otherwise
       TuningParams.Cd_ref = zeros(size(TuningParams.t_ref));
       TuningParams.Cl_ref = zeros(size(TuningParams.t_ref));
end









