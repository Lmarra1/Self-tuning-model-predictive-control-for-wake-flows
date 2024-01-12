%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open Access MATLAB Code

%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
% (RUN THE CODE SETUP.m BEFORE USING MPC_MAIN.m) %
%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Luigi Marra
% Last Modified: TBD
% Version: 1.0.0
%---------------------------------------
% Description: 
% The application of model predictive control (MPC) is proposed, where control parameters are not manually selected 
% but identified through a Bayesian optimization method. The chosen control plant is the chaotic wake of the 
% fluidic pinball at Reynolds number Re = 150. A plant model is obtained using the SINDYc technique. 
% Control is applied for drag reduction and lift stabilization, challenged by the presence of measurement noise in the control sensors. 
% The presence of noise is mitigated by using the Local Polynomial Regression technique.
% --
% In this case the plant does not correspond to the DNS simulation of the fluidic pinball wake 
% but to the plant model obtained with the SINDYc technique. 
% This model is also used for prediction purposes.
% 
%---------------------------------------
% Contact Information:
% - Email:    lmarra@pa.uc3m.es
%---------------------------------------
% Repository URL: 
% https://github.com/Lmarra1/Self-tuning-model-predictive-control-for-wake-flows
%---------------------------------------
% Associated Research Paper:
% - Title:   Self-tuned model predictive control for wake flow
% - Authors: Luigi Marra - Andrea Meilán Vila - Stefano Discetti 
% - Journal: Journal of fluid mechanics
% - Year:    2024
% - DOI:     TBD
%---------------------------------------

%% Run some general settings: clean workspace; set some default properties; add util folders to path
run("Settings")

%% Get unforced Fluidic Pinball data (Cd and Cl)
load("UnforcedPinballData.mat")

%% Flags for different conditions of MPC
% Set boolean flags for various conditions affecting the MPC algorithm.
% These include adding noise to training data, applying the smoothing technique local polynomial regression (LPR),
% considering measurement noise during MPC, tuning control hyperparameters using Bayesian Optimization.

% AddNoiseToTrain      = false;         % Switch to "true" if you want to add noise to the training dataset before training the plant model.
% UseLPRTraining       = false;         % Switch to "true" if you want to apply the smoothing technique "LPR" to the trining dataset (only in the case of measurements noise)
AddNoise               = true;          % Switch to "true" if you want to consider measurement noise during MPC implementation
UseLPR                 = true;          % Switch to "true" if you want to apply LPR to online state measurements during MPC implementation

PerformTuningParams    = true;         % Switch to "true" if you want to tune the control hyperparameters using Bayesian optimization (BO)
UseLPRtuning           = true;         % Switch to "true" if you want to use LPR during the control tuning procedure
IncludeNtuning         = true;         % Switch to "true" if you want to include the prediction/control window length in the tuning procedure

% Some compatibility checks in the flag variables...
if ~AddNoise;            UseLPR         = false;  UseLPRtuning = false;  end
if ~PerformTuningParams; IncludeNtuning = false;  UseLPRtuning = false;  end


%% Noise on the variables
FullScaleCd          = max(abs(Cd_unforced));
FullScaleCl          = max(abs(Cl_unforced));
NoiseLevel           = 1;                % The noise is given in percentage of unforced Cd/Cl full scales

Noise_Cd             = NoiseLevel/100*FullScaleCd * double(AddNoise); 
Noise_Cl             = NoiseLevel/100*FullScaleCl * double(AddNoise);       
Sigma                = [Noise_Cd, Noise_Cl].';


%% Training of the method
% Load a pre-trained SINDY model for the fluidic pinball wake. 
load("SindycModel.mat")

% The Xi matrix denotes the coefficients matrix on the right-hand side (rhs) of the SINDYc force model 
% for the fluidic pinball.

% The state vector is a = [Cd, Cl, dCd_dt, dCl_dt]
% The input vector is b = [b_1, b_2, b_3]
% where: 1 - Front cylinder,   2 - Top cylinder, 3 - Bottom cylinder

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%  MPC  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

%% Parameters MPC
NumbOfCyls   =  3;                     % Number of rotating cylinders
dt           =  0.1;                   % Time-step of the Fluidic Pinball wake Solver
Ts           =  0.5;                   % Control timestep (every Ts c.u. the MPC updates the actuation vector).
Duration     =  200;                   % Duration of the application of control in c.u.
StartControl =  50;                    % Time-delay of free Fluidic Pinball (MPC starts at "StartControl" c.u. otherwise b0 is applied as actation)

Alpha_max    =  8;                     % Max angular acceleration of rear cylinders in D/(c.u.)²
b0           =  zeros(3,1);            % Initial actuation
a0           =  [3.46 0.1 0 0].';      % Initial condition of the state vector


% Options for MPC online optimization
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',500);

%% Other variables to set
Nt         =  (Duration/dt)+1;         % Number of total timestep considered for the MPC implementation
StartLPR   =  StartControl;            % Time to wait in c.u. before applying the LPR technique on state measurements
T          =  0;                       % Initial control timestep
c          =  a0(1:2) + randn(size(a0(1:2))) .* Sigma * double(AddNoise);


%% Select Parameters of the MPC: Hand selection or Tuning with BayesOpt
run('getMPCparams.m');


%% Initialize storage variables
a_History           =  nan(4,Nt);           a_History(:,1)       =   a0;
c_History           =  nan(2,Nt);           c_History(:,1)       =   c;
b_History           =  nan(3,Nt);           b_History(:,1)       =   b0;
t_History           =  nan(1,Nt);           t_History(1)         =   0;
c_Deriv_LPR_History =  nan(2,Nt);
c_LPR_History       =  nan(2,Nt);
cstar_History       =  nan(2,Nt);
a                   =  nan(4,Ts/dt+1);
J                   =  nan(1, (Duration/Ts)+1);

%% Start simulation
disp("Simulation started, check the online plots for the progress status...")

for j = 1:(Duration/Ts)
    tic;
    if j == 1
        a(:,1) = a0;
    else
        a(:,1) = a(:,end);
    end

    %% Setting of the reference state to follow
    cstar = ReferenceTrajectory(T, StartControl);


    %% Decision (with control ON/OFF) of the fluidic pinball actuation vector
    % If the control is OFF the actuation b0 is used 
    % otherwise it is selected by MPC

    if T < StartControl
        b_opt    =  repmat(b0,1,N);
    else


        % Definition of the initial condition for prediction. Since the
        % derivatives are not directly measured (it is possible to measure only C_d and C_l) 
        % they are obtained either with LPR or with finite differences
        if UseLPR && T >= StartLPR
            c_Deriv = c_Deriv_LPR_History(:,Ts/dt*(j-1)+1);
            Pred_IC   = [c_LPR_History(:,Ts/dt*(j-1)+1); c_Deriv(:)];
        else
            for ii = 2:-1:1
                c_Deriv(ii)      = CalcDeriv(t_History(1:Ts/dt*(j-1)+1), c_History(ii,1:Ts/dt*(j-1)+1), length(1:Ts/dt*(j-1)+1));
            end
            Pred_IC   = [c(:,end); c_Deriv(:)];
        end


        %%Define the cost functional as an anonymous MATLAB function
        J_MPC = @(b) J_MPC_func(b, Pred_IC, cstar, b_opt(:,1), params, Xi, dt);

        %%Reading of the past actuation value
        b_old = b_opt(:,1);

        %%Solution of the optimal problem and update of the optimal input sequence
        [b_opt,J(j)] = fmincon(J_MPC,b_opt,[],[],[],[],LB,UB, @(u) InputNonLynConstraint(u,b_old,Alpha_max,dt),options);
    end



    %% Plant integration
    for tt = 1:Ts/dt
        a(:,tt+1) = rk_SINDYc(a(:,tt), b_opt(:,1), dt, 1, Xi, 2);
    end


    %% Measurement of the state vector
    c = a(1:2,:) + randn(size(a(1:2,:))) .* Sigma * double(AddNoise);


    %% Store results of the current MPC iteration
    index = Ts/dt*(j-1)+2:Ts/dt*j + 1;
    a_History(:,index)      =  a(:,2:end);
    c_History(:,index)      =  c(:,2:end);
    b_History(:,index)      =  repmat(b_opt(:,1),1,Ts/dt);
    t_History(index)        =  Ts*(j-1)+dt:dt:Ts*j;
    cstar_History(:,index)  =  repmat(cstar,1,Ts/dt);


    %% LPR application on state measurement (if needed)
    % LPR is applied asymmetrically since only present/past 
    % measurements of the state vector are available. 
    % LPR is performed starting from "StartLPR" c.u.

    if UseLPR && AddNoise
        index = 1:Ts/dt*j+1;
        if T == StartLPR-Ts
            [LPRc, LPRcderiv]               = RunLPR_R(t_History(index).', c_History(:,index).', t_History(index).');
            c_LPR_History(:,index)          = LPRc.';
            c_Deriv_LPR_History(:,index)    = LPRcderiv.';

        elseif T >= StartLPR
            [LPRc, LPRcderiv]    = RunLPR_R(t_History(index).', c_History(:,index).', t_History(index).');
            LPRc       = LPRc(end-Ts/dt+1:end,:);
            LPRcderiv  = LPRcderiv(end-Ts/dt+1:end,:);

            index = Ts/dt*(j-1)+2:Ts/dt*j + 1;
            c_LPR_History(:,index)       = LPRc.';
            c_Deriv_LPR_History(:,index) = LPRcderiv.';

        end
    end


    %% Update time and continue
    T = j*Ts;
    el_time = toc;
    if mod(T,10) == 0
        disp("Control timestep T = " + string(T) + "c.u.  // Elapsed time during the current iteration: " + string(toc));
    end

    %% Plot some data online (states, actuation etc...)
    Case =  "Control";
    run("OnlinePlot.m")
end
