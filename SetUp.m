close all; clear; clc

%% INSERT HERE THE FULL PATH OF Rscript.exe IN YOUR DEVICE AFTER THE INSTALLATION

RscriptPath = '...PASTE IT HERE...';
% e.g.  RscriptPath = 'C:\Program Files\R\R-4.2.2\bin\Rscript.exe';

%%
FolderLPRPath = strcat(pwd,'/utils_LPR');
FolderLPRPath = strrep(FolderLPRPath, '\', '/');
fid = fopen('utils_LPR/CodeR_LPR_.txt', 'r');
file_content = fread(fid, '*char')'; 
fclose(fid);
new_content = strrep(file_content, '*MYPATH*', FolderLPRPath);
fid = fopen('utils_LPR/CodeR_LPR.R', 'w');
fprintf(fid, '%s', new_content);
fclose(fid);


%%
RscriptPath = strrep(RscriptPath, '/', '\');
fid = fopen('utils_LPR/RunLPR_R_.txt', 'r');
file_content = fread(fid, '*char')'; 
fclose(fid);
new_content = strrep(file_content, '*RSCRIPTPATH*', RscriptPath);
fid = fopen('utils_LPR/RunLPR_R.m', 'w');
fprintf(fid, '%s', new_content);
fclose(fid);


%%
command = [RscriptPath, ' -e "install.packages(''lokern'')"'];
status = system(command);

%% Check...
if status == 0
    disp('Package "lokern" installed successfully.');
else
    disp('Error installing package "lokern".');
end

if exist('utils_LPR/CodeR_LPR.R') && exist('utils_LPR/RunLPR_R.m')
disp('---SetUp done successfully.');
disp('---You can run now "MPC_main!"');
disp('...ENJOY!');
end

