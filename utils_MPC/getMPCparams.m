%% Parameters needed for MPC
%%% Weights for the state components inside  the prediction horizon
%----->  Q    =  [Q1, Q2];

%%% Penalization of Input Variability 
%----->  Rdu  =  [Rdu1, Rdu2, Rdu3];

%%% Penalization of Input
%----->  Ru   =  [Ru1, Ru2, Ru3];

%%% Weights for the state components at the end of the prediction horizon
%----->  Qj   =  Q;

%%% Upper and Lower bound of rotational speed of the three cilinders 
%----->  LB = [LB_1, LB_2, LB_3];   Lower Bound of control input
%----->  UB = [UB_1, UB_2, UB_3];   Upper Lower Bound of control input

%%% Length of prediction/control window 
%----->  N


if ~IncludeNtuning
    N    =  3/dt;
end
LB   =  -1;             % Lower Bound of control input
UB   =   1;             % Upper Bound of control input


%% Parameters to be tuned or hand selected

if PerformTuningParams

    % Extended case: 
    % eta = [Q1, Q2, Rdu1, Rdu2, Rdu3, Ru1, Ru2, Ru3]
    % ExtendedSym:
    % eta = [Q1, Q2, Rd1, Rdu23, Ru1, Ru23]
    % Reduced
    % eta = [Q1, Q2, Rdu, Ru]
    
    % If IncludeNtuning is "true" also N is included

    TuningParams.TuningCase     = "Extended"; 
    TuningParams.Duration       = 15;             % Length of each MPC offline simulation 
    TuningParams.DT             = 8;              % Time (in c.u.) starting from the end of each simulation related to data used to calculate the cost functional for tuning
    TuningParams.StartControl   = 4;              % Every sampling of the Cost Functional to tune eta-parameters, run TnoControl c.u. of free simulation

    % Params Case Extended
    % Area of minimum search for tuning
    TuningParams.Q1_lim    = [0    10];
    TuningParams.Q2_lim    = [0    10];
    TuningParams.Ru1_lim   = [0    10];
    TuningParams.Ru23_lim  = [0    10];
    TuningParams.Rdu1_lim  = [0    10];
    TuningParams.Rdu23_lim = [0    10];
    TuningParams.N_lim     = [1     4];
    
    % Params Case Reduced
    TuningParams.Ru_lim    = [0    10];
    TuningParams.Rdu_lim   = [0    10];
    
    % General Params for Bayesian Optimization
    TuningParams.AcquisitionFunction      = 'expected-improvement';    % Acquisition function to decide where to sample next
    TuningParams.MaxObjectiveEvaluations  = 30;                        % Max number of sampling the cost function
    TuningParams.NumSeedingPt             = 15;                        % Number of initial points to span the function to be minimized
    
    run("BayesOpt_MPC_FluidicPinball.m");                              % Run the Bayesian algorithm to tune the eta parameters
    
else
    N   =  3/dt;
    Q   =  [4 2];
    Rdu =  [1 1 1]/3;
    Ru  =  [1 1 1]/3;
end

%%% Bounds of the exogenous input in the prediction window
% LB   =  -1*ones(N,3);            % Lower Bound of control input
% UB   =   1*ones(N,3);            % Upper Bound of control input

[params.N, params.Q, params.Ru, params.Rdu] = deal(N, Q, Ru, Rdu);

LB = repmat(LB,1,N);
UB = repmat(UB,1,N);


%% Default quantities in case of errors in the defined variables
%--Q--
if length(Q) ~= 2
    Q = ones(1,2);
    disp("-----> Q set to default value...")
end


%--Rdu--
if length(Rdu) ~= NumbOfCyls
    Rdu = ones(1,NumbOfCyls);
    disp("-----> Rdu set to default value...")
end

%--Ru--
if length(Ru) ~= NumbOfCyls
    Ru = ones(1,NumbOfCyls);
    disp("-----> Ru set to default value...")
end
