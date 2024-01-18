



%% Some checks on variables...
if ~(mod(Ts, 0.05) == 0)
    Ts = max(round(Ts/0.05)*0.05, 0.05);
    disp(['Error: Ts must be a positive multiple of 0.05. Ts = ', num2str(Ts), ' selected']);
end



% Check for valid dt
if ~(dt > 0 && dt ~= Ts) || ~(mod(Ts,dt) == 0)
    dt = Ts/3;
    disp(['Error: dt must be a positive divisor of Ts. dt = Ts/5 = ', num2str(dt), ' selected']);
end


% Check for valid Duration
if ~(mod(Duration,Ts) == 0)
    Duration = round(Duration/Ts)*Ts;
    disp(['Error: Duration must be a positive multiple of Ts. Duration = ', num2str(Duration), ' selected']);
end

% Check for valid Duration in BO tuning
if ~(mod(TuningParams.Duration,Ts) == 0)
    TuningParams.Duration = round(TuningParams.Duration/Ts)*Ts;
    disp(['Error: Duration in BO must be a positive multiple of Ts. Duration = ', num2str(Duration), ' selected']);
end


% Check for valid StartControl
if ~(mod(StartControl,Ts) == 0)
    StartControl = round(StartControl/Ts)*Ts;
    disp(['Error: StartControl must be a positive multiple of Ts. StartControl = ', num2str(StartControl), ' selected']);
end


% Check for valid StartControl in BO tuning
if ~(mod(TuningParams.StartControl,Ts) == 0)
    TuningParams.StartControl = round(TuningParams.StartControl/Ts)*Ts;
    disp(['Error: StartControl in BO must be a positive multiple of Ts. StartControl = ', num2str(StartControl), ' selected']);
end


% Check for valid constraints
if ~(LB <= UB)
    disp('Error: Lower bound (LB) must be less than or equal to the upper bound (UB). LB = -1 and UB = 1 selected');
    LB = -1;
    UB = 1;
end

if ~(Alpha_max >= 0)
    disp('Error: AlphaMax must be a non-negative value. AlphaMax = 8 selected');
    Alpha_max = 8;
end


if ~(Duration >= StartControl)
    disp('Error: Duration must be bigger than StartControl. StartControl = round(Duration/4) selected');
    StartControl = round(Duration/4);
end



% Check for valid PerformTuningParams
if ~islogical(PerformTuningParams)
    disp('Error: PerformTuningParams must be a logical value. False selected');
    PerformTuningParams = false;
end


% Check for valid UseLPRtuning
if ~islogical(AddNoise)
    disp('Error: AddNoise must be a logical value. False selected');
    AddNoise = false;
end


% Check for valid UseLPRtuning
if ~islogical(UseLPRtuning)
    disp('Error: UseLPRtuning must be a logical value. False selected');
    UseLPRtuning = AddNoise;
end

% Check for valid IncludeNtuning
if ~islogical(IncludeNtuning)
    disp('Error: IncludeNtuning must be a logical value. True selected');
    IncludeNtuning = true;
end

% Check for valid tuning case
if ~(any(strcmp(TuningParams.TuningCase, {'Extended', 'ExtendedSym', 'Reduced', 'extended', 'extendedsym', 'reduced', 'extendedSym'})))
    disp('Error: Invalid TuningCase value. ExtendedSym selected');
    TuningParams.TuningCase = "ExtendedSym"; % Assign default value
end


% Check for simulation Duration in BO tuning
if ~(TuningParams.Duration >= TuningParams.StartControl)
    disp('Error: Duration must be bigger than StartControl in BO tuning. StartControl = round(Duration/4) selected');
    TuningParams.StartControl = round(TuningParams.Duration/4);
end


% Check for DT in BO tuning
if ~(TuningParams.Duration - TuningParams.StartControl >= TuningParams.DT)
    disp('Error: You are considering also the free pinball forces to sample the J_BO for tuning parameters. DT = Duration - StartControl selected');
    TuningParams.DT = TuningParams.Duration - TuningParams.StartControl;
end


% Check for NumSeedingPt
if ~(TuningParams.MaxObjectiveEvaluations >= TuningParams.NumSeedingPt)
    disp('Error: Selected more BO iteration. Too high NumSeedingPt');
    TuningParams.MaxObjectiveEvaluations  = TuningParams.NumSeedingPt + 20;
end


% Check for PlotVerbose
if ~(any(MPCPlotVerbose == [0 1 2]))
    disp('Error: Wrong PlotVerbose. PlotVerbose = 2 selected');
    MPCPlotVerbose =  2;
end


% Check for manual selection parameters if tuning is not performed
if ~PerformTuningParams
    if ~(N > 0)
        disp('Error: N must be a positive value when PerformTuningParams is false.');
        N = 3/dt;
    end

    if ~(all(Q > 0) && length(Q) == 2)
        disp('Error: All elements of Q must be positive when PerformTuningParams is false.');
        Q = [4, 2]; 
    end

    if ~(all(Rdu >= 0) && length(Rdu) == 3)
        disp('Error: All elements of Rdu must be non-negative when PerformTuningParams is false.');
        Rdu = [1, 1, 1]/3; 
    end

    if ~(all(Ru >= 0) && length(Ru) == 3)
        disp('Error: All elements of Ru must be non-negative when PerformTuningParams is false.');
        Ru = [1, 1, 1]/3; 
    end
end


if ~AddNoise
    UseLPR         = false;
    UseLPRtuning   = false;
end

if ~PerformTuningParams
    IncludeNtuning = false;
    UseLPRtuning   = false;
end

