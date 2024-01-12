function cstar = ReferenceTrajectory(T, StartControl)

%% Description:
% This function is used to specify the target trajectory of the control.
% A constant value can be entered for a setpoint stabilization problem
% or a time dependent value for a trajectory tracking case.

%%% Inputs:
% - T: Time in convective units (c.u.)
% - StartControl: Time in c.u. in which the control starts

%%% Outputs:
% - cstar: Reference trajectory for the controllable features vector.
% The first component concerns the C_d reference trajectory
% while the second refers to the C_l

% Some example cases are presented below:
% - Case 1: C_d minimization and C_l stabilization around null value
% - Case 2: C_d minimization and C_l tracking to a step function
% - Case 3: C_d stabilization around 2.6 and C_l tracking to a step function


TrajCase =  2;

switch TrajCase
    case 1
        if T<StartControl
            cstar = nan(2,1);
        else
            cstar = [0;0];
        end

        %%%%%%%%%%%%%%%%
    case 2
        if T<StartControl
            cstar = nan(2,1);
        elseif T>=50 && T< 75
            cstar = [0 ;  -1.3];
        elseif T>=75 && T< 100
            cstar = [0 ;  -0.5];
        elseif T>=100 && T<125
            cstar = [0 ;  -1];
        elseif T>=125 && T<150
            cstar = [0 ;  1];
        elseif T>=150 && T<175
            cstar = [0 ;  0];
        else
            cstar = [0 ; -0.9];
        end

        %%%%%%%%%%%%%%%%
    case 3
        ttt = 0:0.1:200;
        if T<StartControl
            cstar = nan(2,1);
        else
            cstar = [2.6 ; interp1(ttt, sin((ttt-50)/150*2*pi).*(ttt>50)*1.5 , T , "linear")];
        end
end

end