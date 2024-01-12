function cstar = ReferenceTrajectory_tuning(T,StartControl)

%% Description:
% This function is used to specify the target trajectory of the MPC control
% during the tuning parameter process
% A constant value can be entered for a setpoint stabilization problem
% or a time dependent value for a trajectory tracking case.

%%% Inputs:
% - T:             Time in convective units (c.u.)
% - StartControl:  Time in c.u. in which the control starts

%%% Outputs:
% - cstar:      Reference trajectory for the controllable features vector.
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
        elseif T>=8 && T< 12
            cstar = [3 ;  1.5];
        else
            cstar = [3 ;  -1.5];
        end


        %%%%%%%%%%%%%%%%
    case 3
        ttt = 0:0.1:30;
        if T<StartControl
            cstar = nan(2,1);
        else
            cstar = [2.6 ; interp1(ttt, sin((ttt-4)/26*2*pi).*(ttt>4)*1.5 , T , "spline")];
        end

end













%  xref = [0;0];
% 
%     if T<8
%         xref = [3 ;  0];
%     elseif T>=8 && T< 18
%         xref = [3 ;  2.5];
%     else
%         xref = [3 ;  -2.5];
%     end
    





end