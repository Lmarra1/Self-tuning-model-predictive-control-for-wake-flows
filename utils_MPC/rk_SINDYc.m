function an = rk_SINDYc(a0, b0, dt, nstep, Xi, RKorder)

%% Description:
% Function to integrate the SINDYc plant model of fluidic pinball 

%%% Inputs:
% - a0:       Initial state vector
% - b0:       Input vector 
% - b0:       Value of the input vector at the previous actuation
% - Xi:       Coefficient matrix on the right-hand side (rhs) of the SINDY model of the plant
% - dt:       Time interval for integration
% - nstep:    Number of integration step
% - RKorder:  Runge-Kutta order for plant model integration

%%% Outputs:
% - an:       State vector after nstep*dt 


an = a0;

% Forward Euler
if RKorder == 1
    for i = 1:nstep
        an = rhs_SINDYc(an, b0, Xi) * dt + an;
    end

% Second-order method
elseif RKorder == 2
    alpha = 1;
    alphadt = alpha * dt;
    [c1, c2] = deal((1 - 1/(2*alpha)) * dt, 1/(2*alpha) * dt);
    for i = 1:nstep
        k1 = rhs_SINDYc(an, b0, Xi);
        k2 = rhs_SINDYc(an + alphadt * k1, b0, Xi);
        an = (k1 * c1 + k2 * c2) + an;
    end

    % Third-order method
elseif RKorder == 3
    [c1, c2, c3, dtm, dt2] = deal((1/6) * dt, (2/3) * dt, (1/6) * dt, 0.5 * dt, 2 * dt);
    for i = 1:nstep
        k1 = rhs_SINDYc(an, b0, Xi);
        k2 = rhs_SINDYc(an + dtm * k1, b0, Xi);
        k3 = rhs_SINDYc(an - dt * k1 + dt2 * k2, b0, Xi);
        an = (c1 * k1 + c2 * k2 + c3 * k3) + an;
    end

    % Fourth-order method
elseif RKorder == 4
    [dtm, dts] = deal(0.5 * dt, (1/6) * dt);
    for i = 1:nstep
        k1 = rhs_SINDYc(an, b0, Xi, len_a, len_b);
        k2 = rhs_SINDYc(an + dtm * k1, b0, Xi);
        k3 = rhs_SINDYc(an + dtm * k2, b0, Xi);
        k4 = rhs_SINDYc(an + dt * k3, b0, Xi);
        an = (k1 + 2 * k2 + 2 * k3 + k4) * dts + an;
    end

else
    disp('Selected RK not provided in the library: change the order...');
end
end






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rhs = rhs_SINDYc(a, b, Xi)
%% Description:
% Function to evaluate the rhs of the plant model of the fluidic pinball.
% The model includes a library of polynomial functions up to the second order.

%%% Inputs:
% - a:       State vector
% - b:       Input vector 
% - Xi:      Coefficient matrix on the right-hand side (rhs) of the SINDY model of the plant


%%% Outputs:
% - rhs:     Plant model right-hand side 

%%

% poly order 1
ab = [a(:); b(:)].';
n  = length(ab);
rhs2 = nan(1,(n+1)*n/2);

% poly order 2
ind = 1;
for i=1:n
    for j=i:n
        rhs2(ind) = ab(i).*ab(j);
        ind = ind+1;
    end
end


rhs = [1, ab, rhs2];

rhs = (rhs * Xi).';

end