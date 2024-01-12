function [Q, Rdu, Ru, N] = ReadTuningParamsfromEta(Eta)

%% Description:
% This function extracts tuning parameters (Q, Rdu, Ru, N) from the parameter vector Eta.

%%% Inputs:
% - Eta: Parameter vector containing tuning parameters.
%   - For "Reduced"      Eta = [Q1, Q2, Ru, Rdu, N]
%   - For "ExtendedSym"  Eta = [Q1, Q2, Ru1, Ru23, Rdu1, Rdu23, N]
%   - For "Extended"     Eta = [Q1, Q2, Ru1, Ru2, Ru3, Rdu1, Rdu2, Rdu3, N]


%%% Outputs:
% - Q:       Weighting matrix for state errors
% - Rdu:     Weighting matrix for control input rate of change
% - Ru:      Weighting matrix for control input
% - N:       Prediction horizon

%%
Eta = Eta(:).';

if any(length(Eta) == [5 7 9])
    N = Eta(end);
    Eta(end) = [];
else
    N = [];
end

if length(Eta) == 4
    Q   =  Eta(1:2);
    Ru  =  Eta(3) .* ones(1,3);
    Rdu =  Eta(4) .* ones(1,3);
elseif length(Eta) == 6
    Q   =  Eta(1:2);
    Ru  = [Eta(3)  Eta(4)*ones(1,2)];
    Rdu = [Eta(5)  Eta(6)*ones(1,2)];
else
    Q   =  Eta(1:2);
    Ru  =  Eta(3:5);
    Rdu =  Eta(6:8);
end

end



