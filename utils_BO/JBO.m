function J_BO = JBO(eta,Param)

%% Description:
% This function runs an MPC simulation using a specific set of control parameters.
% It is used to sample the cost functional J_BO (based on MPC control results) during Bayesian optimization.

%%% Inputs:
% - eta:    Parameter vector of the current optimization step
% - Param:  Struct containing various parameters

%%% Outputs:
% - J_BO:   Sampled cost functional J_BO.




%% Read needed Parameters
dt                      = Param.dt;
Ts                      = Param.Ts;
Duration                = Param.Duration;
StartControl            = Param.StartControl;
StartLPR                = StartControl;
AddNoise                = Param.AddNoise;
LB                      = Param.LB;
UB                      = Param.UB;
options                 = Param.options;
Alpha_max               = Param.Alpha_max;
Sigma                   = Param.Sigma;
a0                      = Param.a0;
b0                      = Param.b0;
UseLPR                  = Param.UseLPRtuning;
IncludeNtuning          = Param.IncludeNtuning;
Xi                      = Param.Xi;
MPCPlotVerbose          = Param.MPCPlotVerbose;
t_ref                   = Param.t_ref;
Cd_ref                  = Param.Cd_ref;
Cl_ref                  = Param.Cl_ref;
Feedcstartype           = Param.Feedcstartype;


%% Get tuning Parameters for current optimization step
[Q, Rdu, Ru, N] = ReadTuningParamsfromEta(table2array(eta));

if ~IncludeNtuning
    N = Param.N;
else
    N = round(N/dt);
end

[params.N, params.Q, params.Ru, params.Rdu] = deal(N, Q, Ru, Rdu);
LB = repmat(LB,1,N);
UB = repmat(UB,1,N);

%% Other variables to set
Nt         =  round((Duration/dt))+1;         % Number of Total Timestep of pinball-code calcs.
c          =  a0(1:2) + randn(size(a0(1:2))) .* Sigma * double(AddNoise);
T          =  0;
Ts_dt      =  round(Ts/dt);

%% Initialize storage variables
a_History           =  nan(4,Nt);           a_History(:,1)       =   a0;
c_History           =  nan(2,Nt);           c_History(:,1)       =   c;
b_History           =  nan(3,Nt);           b_History(:,1)       =   b0;
t_History           =  nan(1,Nt);           t_History(1)         =   0;
c_Deriv_LPR_History =  nan(2,Nt);
c_LPR_History       =  nan(2,Nt);
cstar_History       =  nan(2,Nt);

%% Start simulation
tic
for j = 1:(Duration/Ts)

    if j == 1
        a(:,1) = a0;
    else
        a(:,1) = a(:,end);
    end

    %% Set reference state to follow

    if logical(Feedcstartype)
        cstar = interp1(t_ref,[Cd_ref;Cl_ref].',T,"linear").';
    else
        cstar = interp1(t_ref,[Cd_ref;Cl_ref].',T:dt:T+(N-1)*dt,"linear").';
    end


    %% Perform (control on/off) calculations for optimal input vector
    if T < StartControl
        b_opt    =  repmat(b0,1,N);
    else

        if UseLPR && T >= StartLPR
            c_Deriv = c_Deriv_LPR_History(:,Ts_dt*(j-1)+1);
        else
            %Calculate states derivative with finite differences
            for ii = 2:-1:1
                c_Deriv(ii)      = CalcDeriv(t_History(1:Ts_dt*(j-1)+1), c_History(ii,1:Ts_dt*(j-1)+1), length(1:Ts_dt*(j-1)+1));
            end
        end


        %%Define the IC for the prediction in the Optimal Problem
        if UseLPR && T >= StartLPR
            Pred_IC   = [c_LPR_History(:,Ts_dt*(j-1)+1); c_Deriv(:)];
        else
            Pred_IC   = [c(:,end); c_Deriv(:)];
        end


        %%Define the cost functional as anon. MATLAB function
        J_MPC = @(b) J_MPC_func(b, Pred_IC, cstar, b_opt(:,1), params, Xi, dt);

        %%Read the past value of the input
        b_old = b_opt(:,1);

        %%Solve the optimal problem and update the optimal input sequence
        b_opt = fmincon(J_MPC,b_opt,[],[],[],[],LB,UB, @(u) InputNonLynConstraint(u,b_old,Alpha_max,dt),options);
    end



    %% Plant integration
    for tt = 1:Ts_dt
        a(:,tt+1) = rk_SINDYc(a(:,tt), b_opt(:,1), dt, 1, Xi, 2);
    end


    %% Perform measurement of the state vector
    c = a(1:2,:) + randn(size(a(1:2,:))) .* Sigma * double(AddNoise);


    %% Save variables
    index = Ts_dt*(j-1)+2:Ts_dt*j + 1;
    a_History(:,index)       =  a(:,2:end);
    c_History(:,index)       =  c(:,2:end);
    b_History(:,index)       =  repmat(b_opt(:,1),1,Ts_dt);
    t_History(index)         =  Ts*(j-1)+dt:dt:Ts*j;
    if size(cstar,2) == N
        cstar_History(:,index)  =  cstar(:,1:Ts_dt);
    else
        cstar_History(:,index)  =  repmat(cstar,1,Ts_dt);
    end

    %%%


    %% Calculate LPR of the state
    if UseLPR && AddNoise
        index = 1:Ts_dt*j+1;
        if T == StartLPR-Ts
            [LPRc, LPRcderiv]          = RunLPR_R(t_History(index).', c_History(:,index).', t_History(index).');
            c_LPR_History(:,index)                = LPRc.';
            c_Deriv_LPR_History(:,index)          = LPRcderiv.';

        elseif T >= StartLPR
            [LPRc, LPRcderiv]    = RunLPR_R(t_History(index).', c_History(:,index).', t_History(index).');
            LPRc       = LPRc(end-Ts_dt+1:end,:);
            LPRcderiv  = LPRcderiv(end-Ts_dt+1:end,:);

            index = Ts_dt*(j-1)+2:Ts_dt*j + 1;
            c_LPR_History(:,index)       = LPRc.';
            c_Deriv_LPR_History(:,index) = LPRcderiv.';

        end
    end


    %% Update time and continue
    T = j*Ts;

    %% Plot some data online (states, actuation etc...)
    if any(MPCPlotVerbose == [1 2]) && T < Duration
        run("OnlinePlot.m")
    end

end

if MPCPlotVerbose == 0
    run("OnlinePlot.m")
end


%%%

if UseLPR && AddNoise
    StateHist   = RunLPR_R(t_History.', c_History.', t_History.');
    StateHist   = StateHist.';
else
    StateHist   = c_History;
end


%% Evaluate the cost functional
J_BO = evalJBO(StateHist(:,1:Ts_dt:end),cstar_History(:,1:Ts_dt:end),b_History(:,1:Ts_dt:end),Param.DT,Ts);

end
