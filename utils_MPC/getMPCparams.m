%% Parameters needed for MPC
%%% Weights for the state components errors
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


%% Prepare data to run the Bayesian Optimization

if PerformTuningParams

    %%
    if strcmp(TuningParams.TuningCase,"Extended") || strcmp(TuningParams.TuningCase,"extended")
        
        % State penalization weights
        Q1   = optimizableVariable("Q1"   , TuningParams.Q1_lim);
        Q2   = optimizableVariable("Q2"   , TuningParams.Q2_lim);

        % Input Penalization weights
        Ru1   = optimizableVariable("Ru1"   , TuningParams.Ru1_lim);
        Ru2   = optimizableVariable("Ru2"   , TuningParams.Ru23_lim);
        Ru3   = optimizableVariable("Ru3"   , TuningParams.Ru23_lim);

        % Input Variability Penalization weights
        Rdu1  = optimizableVariable("Rdu1"  , TuningParams.Rdu1_lim);
        Rdu2  = optimizableVariable("Rdu2"  , TuningParams.Rdu23_lim);
        Rdu3  = optimizableVariable("Rdu3"  , TuningParams.Rdu23_lim);

        % Create the vector of variable to be tuned
        Eta = [Q1, Q2, Ru1, Ru2, Ru3, Rdu1, Rdu2, Rdu3];

%%
    elseif strcmp(TuningParams.TuningCase,"ExtendedSym") || strcmp(TuningParams.TuningCase,"extendedSym")

        % State penalization weights
        Q1   = optimizableVariable("Q1"   , TuningParams.Q1_lim);
        Q2   = optimizableVariable("Q2"   , TuningParams.Q2_lim);

        % Input Penalization weights
        Ru1   = optimizableVariable("Ru1"   , TuningParams.Ru1_lim);
        Ru23  = optimizableVariable("Ru23"  , TuningParams.Ru23_lim);

        % Input Variability Penalization weights
        Rdu1  = optimizableVariable("Rdu1"  , TuningParams.Rdu1_lim);
        Rdu23 = optimizableVariable("Rdu23" , TuningParams.Rdu23_lim);

        % Create the vector of variable to be tuned
        Eta = [Q1, Q2, Ru1, Ru23, Rdu1, Rdu23];



%%
    elseif strcmp(TuningParams.TuningCase,"Reduced") || strcmp(TuningParams.TuningCase,"reduced")
        % State penalization weights
        Q1   = optimizableVariable("Q1"   , TuningParams.Q1_lim);
        Q2   = optimizableVariable("Q2"   , TuningParams.Q2_lim);

        % Input Penalization weights
        Ru  = optimizableVariable("Ru"    , TuningParams.Ru_lim);

        % Input Variability Penalization weights
        Rdu = optimizableVariable("Rdu"  , TuningParams.Rdu_lim);

        % Create the vector of variable to be tuned
        Eta = [Q1, Q2, Ru, Rdu];

    end

    % Include length of control/prediction window if needed
    if IncludeNtuning
        Nopt  = optimizableVariable("N"  , TuningParams.N_lim);
        Eta   = [Eta, Nopt];
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
    TuningParams.ControlCase             = ControlCase;
    TuningParams.MPCPlotVerbose          = MPCPlotVerbose;

    if ~IncludeNtuning
        TuningParams.N    =  N;
    end




    %% Run Bayesian Optimization
    disp("Parameter Bayesian optimization started...")
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

end

%%% Bounds of the exogenous input in the prediction window
% LB   =  -1*ones(N,3);            % Lower Bound of control input
% UB   =   1*ones(N,3);            % Upper Bound of control input

[params.N, params.Q, params.Ru, params.Rdu] = deal(N, Q, Ru, Rdu);

LB = repmat(LB,1,N);
UB = repmat(UB,1,N);



