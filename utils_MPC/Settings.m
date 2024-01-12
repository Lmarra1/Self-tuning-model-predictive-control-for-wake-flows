clear; close all; clc;

% --- 
fs     = 15;                                  % Default font size for figures
width  = 14;                                  % Default figure width
lw     = 1.5;                                 % Default linewidth in figures
axislw = 1;                                   % Default axis linewidth for figures
colr   = ["k","r","b"];                       % Colors for plots 


% --- Settings ---
warning off;
set(0,'DefaultFigureColor'    ,'w')         
set(0,'DefaultLineLineWidth'  ,lw)
set(0,'DefaultAxesFontSize'   ,fs)
set(0,'DefaultLegendFontSize' ,fs)
set(0,'DefaultTextFontSize'   ,fs)
set(0,'DefaultAxesLineWidth'  ,axislw)
set(0,'DefaultAxesBox'        ,'on');
set(0,'defaultaxescolor'      ,[1 1 1]);
set(0,'defaultLegendInterpreter'       ,'latex');
set(0,'defaultTextInterpreter'         ,'latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');   

% --- Add util folders to Matlab path  ---
addpath('utils_MPC','utils_LPR','utils_BO','DATA');