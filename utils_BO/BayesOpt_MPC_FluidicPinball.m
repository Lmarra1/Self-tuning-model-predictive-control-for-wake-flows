
%% Prepare data to run the Bayesian Optimization
if strcmp(TuningParams.TuningCase,"Extended") || strcmp(TuningParams.TuningCase,"extended")
    % LENGTH OF ETA IS 8/9
    %% State penalization weights
    Q1   = optimizableVariable("Q1"   , TuningParams.Q1_lim);
    Q2   = optimizableVariable("Q2"   , TuningParams.Q2_lim);
    
    %% Input Penalization weights
    Ru1   = optimizableVariable("Ru1"   , TuningParams.Ru1_lim);
    Ru2   = optimizableVariable("Ru2"   , TuningParams.Ru23_lim);
    Ru3   = optimizableVariable("Ru3"   , TuningParams.Ru23_lim);
    
    %% Input Variability Penalization weights
    Rdu1  = optimizableVariable("Rdu1"  , TuningParams.Rdu1_lim);
    Rdu2  = optimizableVariable("Rdu2"  , TuningParams.Rdu23_lim);
    Rdu3  = optimizableVariable("Rdu3"  , TuningParams.Rdu23_lim);
    
    %% Create the vector of variable to be tuned
    Eta = [Q1, Q2, Ru1, Ru2, Ru3, Rdu1, Rdu2, Rdu3];
   
    
elseif strcmp(TuningParams.TuningCase,"ExtendedSym") || strcmp(TuningParams.TuningCase,"extendedSym")
    % LENGTH OF ETA IS 6/7
    %% State penalization weights
    Q1   = optimizableVariable("Q1"   , TuningParams.Q1_lim);
    Q2   = optimizableVariable("Q2"   , TuningParams.Q2_lim);
    
    %% Input Penalization weights
    Ru1   = optimizableVariable("Ru1"   , TuningParams.Ru1_lim);
    Ru23  = optimizableVariable("Ru23"  , TuningParams.Ru23_lim);
    
    %% Input Variability Penalization weights
    Rdu1  = optimizableVariable("Rdu1"  , TuningParams.Rdu1_lim);
    Rdu23 = optimizableVariable("Rdu23" , TuningParams.Rdu23_lim);

    %% Create the vector of variable to be tuned
    Eta = [Q1, Q2, Ru1, Ru23, Rdu1, Rdu23];
   
    
    
    
elseif strcmp(TuningParams.TuningCase,"Reduced") || strcmp(TuningParams.TuningCase,"reduced") 
    % LENGTH OF ETA IS 4/5
    %% State penalization weights
    Q1   = optimizableVariable("Q1"   , TuningParams.Q1_lim);
    Q2   = optimizableVariable("Q2"   , TuningParams.Q2_lim);
    
    %% Input Penalization weights
    Ru  = optimizableVariable("Ru"    , TuningParams.Ru_lim);
    
    %% Input Variability Penalization weights
    Rdu = optimizableVariable("Rdu"  , TuningParams.Rdu_lim);
    
    %% Create the vector of variable to be tuned
    Eta = [Q1, Q2, Ru, Rdu];

    
else
    disp("There is a mistake in the selection of tuning case. Select Reduced or Extended...");
    return
end

if IncludeNtuning
    Nopt  = optimizableVariable("N"  , TuningParams.N_lim);
    Eta = [Eta, Nopt];
end



%% Read data needed to sample the cost function in the bayesian optimization
TuningParams.Ts                      = Ts;
TuningParams.dt                      = dt;
TuningParams.AddNoise                = AddNoise;
TuningParams.UseLPR                  = UseLPR;
TuningParams.UseLPRtuning            = UseLPRtuning;
TuningParams.IncludeNtuning          = IncludeNtuning;
TuningParams.LB                      = LB;
TuningParams.UB                      = UB;
TuningParams.options                 = options;
TuningParams.Alpha_max               = Alpha_max;
TuningParams.Sigma                   = Sigma;
TuningParams.a0                      = a0;
TuningParams.b0                      = b0;
TuningParams.Xi                      = Xi;

if ~IncludeNtuning
    TuningParams.N    =  N;
end







%% Run Bayesian Optimization
ResultsBayesOpt = bayesopt(@(Eta_var) JBO(Eta_var,TuningParams),Eta,...
    'Verbose',1,...
    'AcquisitionFunctionName' , TuningParams.AcquisitionFunction,...
    'MaxObjectiveEvaluations' , TuningParams.MaxObjectiveEvaluations,...
    'NumSeedPoints'           , TuningParams.NumSeedingPt);

Eta = table2array(bestPoint(ResultsBayesOpt));


%% Read the parameter in output from optimization
[Q,Rdu,Ru,Nopt] = ReadTuningParamsfromEta(Eta);

if IncludeNtuning
    N = round(Nopt/dt);
end

%%











