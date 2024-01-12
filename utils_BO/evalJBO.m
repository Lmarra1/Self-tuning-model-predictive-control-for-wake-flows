function J_BO = evalJBO(cHist,crefHist,uHist,DT,Ts)

%% Description:
% Function to perform a sampling of the cost functional J_BO.

%%% Inputs:
% - cHist:      Historical data of controllable features
% - crefHist:   Historical data of reference controllable features
% - uHist:      Historical data of control inputs
% - DT:         Total time duration of the MPC simulation
% - Ts:         Sampling time

%%% Outputs:
% - J_BO:       Sampled cost functional J_BO.



%% Select initial data
N       =  DT/Ts;
Cd      =  cHist(1,end-N+1:end);
Cl      =  cHist(2,end-N+1:end);
Cdref   =  crefHist(1,end-N+1:end);
Clref   =  crefHist(2,end-N+1:end);
u       =  uHist(:,end-N+1:end);
J_BO =  0;

%% Penalize the controll. performance of Cd/Cl
J_BO = J_BO + mean((Cl-Clref).^2);
J_BO = J_BO + mean((Cd-Cdref).^2);

%% Penalize the variability of input
% J_BO = J_BO + 0 *  1/3*sum([TotalVariation(u(1,:)) TotalVariation(u(2,:)) TotalVariation(u(3,:))])/(N.^2*Ts);

end


%% Function to calculate the Total Variation of a vector
% function TV = TotalVariation(y)
% TV = sum(abs(diff(y)));
% end






