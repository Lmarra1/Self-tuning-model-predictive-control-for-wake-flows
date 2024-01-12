function [c,ceq] = InputNonLynConstraint(b,b0,Alpha_Max,dt)

%% Description:
% This function defines nonlinear constraints for the online optimization problem 

%%% Inputs:
% - b:            Control input
% - b0:           Previous control input.
% - Alpha_Max:    Maximum angular acceleration of the cylinders in [D/(c.u.)^2]
% - dt:           Time interval between control input time steps.

%%% Outputs:
% - c:        Nonlinear constraint value, computed as max(abs(diff(u))) / dt - Alpha_Max, where u represents the control inputs.
% - ceq:      Equality constraint value, set to 0 in this case.

%%
b    = [b0 , b];
c    = max(abs(diff(b,[],2)),[],2)/dt - Alpha_Max;
ceq  = 0;

end


