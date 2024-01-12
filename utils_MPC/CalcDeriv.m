function dy_dx = CalcDeriv(x,y,Index)

%% Description:
% This function calculates the derivative of a given vector 'y' with respect
% to the corresponding vector 'x' using finite differences

%%% Inputs:
% - x:           Independent variable vector
% - y:           Dependent variable vector
% - Index:       (Optional) Index of the point where the derivative is
%                calculated. If not given the derivative is calulated for
%                the entire vector x

%%% Outputs:
% - dy_dx:       Vector of derivatives with respect to 'x'.


%% Pre-processing data
x         =  x(:);
y         =  y(:);
DerivOrd  =  1;
n         =  length(x);
dx        =  x(2) - x(1);
dy_dx     =  nan(size(y));
Nptder    =  7;



%% Check for the length of the vector in input
if n == 1
    error("Impossible to calculate the derivative. More than 1 point needed...")
end

if length(x) ~= length(y)
    error("Impossible to calculate the derivative. x and y of different length...")
end

%% Check for the number of inputs given to the function
if nargin<2
    error("Not enough input arguments...");
end



%%
%If nargin == 2 the function calculates the derivative
% in every point of the domain

%If nargin == 3 the function calculates the derivative
% only in the specified point whose index in the vector is Index


%% Nargin == 2
if nargin ==2

    if n==2
        dy_dx(1:2) = (y(2)-y(1))/dx .* ones(1,2);
    elseif n>2 && n<Nptder
        Nptder = n;
    end

    if mod(Nptder,2)==0
        Nptder = Nptder-1;
    end
    nptext   = (Nptder - 1)/2;


    %Inner points derivative
    InnerWeights = WeigthDerPoly(-nptext:nptext,0,DerivOrd);
    for i=nptext+1:n-nptext
        dy_dx(i) = 1/dx*sum(InnerWeights.*y(i-nptext:i+nptext).');
    end

    %Left points derivative
    Stencil = 1:nptext*2;
    for i = 1:nptext
        Weights = WeigthDerPoly(Stencil,i,DerivOrd);
        dy_dx(i) = 1/dx*sum(Weights.*y(Stencil).');
    end

    %Right points derivative
    Stencil = n-nptext*2+1:n;
    for i = n-nptext+1:n
        Weights = WeigthDerPoly(Stencil,i,DerivOrd);
        dy_dx(i)= 1/dx*sum(Weights.*y(Stencil).');
    end


    %Column vector of derivatives
    dy_dx = dy_dx(:);





    %% Nargin == 3
elseif nargin == 3

    if n==2
        dy_dx(1:2) = (y(2)-y(1))/dx .* ones(1,2);
    elseif n>2 && n<Nptder
        Nptder = n;
    end

    if mod(Nptder,2)==0
        Nptder = Nptder-1;
    end
    nptext   = (Nptder - 1)/2;

    if Index<nptext
        %Left point derivative
        Stencil = 1:nptext;
        Weights = WeigthDerPoly(Stencil,Index,DerivOrd);
        dy_dx = 1/dx*sum(Weights.*y(Stencil).');

    elseif Index >= n-nptext+1
        %Right point derivative
        Stencil = n-nptext+1:n;
        Weights = WeigthDerPoly(Stencil,Index,DerivOrd);
        dy_dx= 1/dx*sum(Weights.*y(Stencil).');

    else
        %Inner point derivative
        Weights = WeigthDerPoly(-nptext:nptext,0,DerivOrd);
        dy_dx = 1/dx*sum(Weights.*y(Index-nptext:Index+nptext).');
    end
end



end

















%% Function to find weights of derivatives
function w=WeigthDerPoly(x,xc,p)
%% FUNCTION LEGEND
%x-----> Stencil
%xc----> Point of derivative evaluation
%p-----> Order of derivative

%% Data for weight derivative estimation
n=length(x);
x=x(:).';
I=eye(n);

%% Centering the stencil
csi=x-xc;

%% Matrix M contruction -> M*w=I
for j=n:-1:1
    M(j,:)=csi.^(j-1)/factorial(j-1);
end

%% Solve the system
w=M\I;
w=w.';

%% Give only the p-th derivative weigths
if nargin==3
    w=w(p+1,:);
end


end



















