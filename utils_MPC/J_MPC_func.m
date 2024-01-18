function J = J_MPC_func(b, a, cstar, b0, params, Xi, dt)

%% Description:
% Cost function of nonlinear MPC

%%% Inputs:
% - a:       State vector 
% - b:       Input vector
% - cstar:   Sampling of the reference trajectory to be followed
% - b0:      Value of the input vector at the previous actuation
% - params:  Struct with parameters necessary for the definition of J_MPC
% - Xi:      Coefficient matrix on the right-hand side (rhs) of the SINDY model of the plant
% - dt:      Time interval for integration

%%% Outputs:
% - J:       Sampling of the cost functional J_MPC

%%
[N, Q, Ru, Rdu] = deal(params.N, params.Q, params.Ru, params.Rdu);

%% Integrate plant model state in the prediction window
Ns = size(a,1);
ak = zeros(Ns,N+1); ak(:,1) = a;

for k=1:N
    ak(:,k+1) = rk_SINDYc(ak(:,k), b(:,k), dt/2, 2, Xi, 2);
end


% Consider only Cd and Cl component from the state vector
ck = ak(1:2,2:N+1);

%% Set Initial Values
bk = b(:,1);
J  = 0;

if size(cstar,2) ~= N
    cstar = repmat(cstar,1,N);
end

%% Loop through each prediction step
for k=1:N

    ck1 = ck(:,k);

    J = J + (ck1-cstar(k))'* diag(Q) *(ck1-cstar(k));

    if k==1
        J = J + (bk-b0)'* diag(Rdu) *(bk-b0) + bk'*diag(Ru)*bk;
    else
        J = J + (bk-b(k-1))'* diag(Rdu) *(bk-b(k-1)) + bk'*diag(Ru)*bk;
    end

    if k<N
        bk = b(:,k+1);
    end

end

end












