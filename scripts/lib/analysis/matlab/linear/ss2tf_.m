% clear workspace 
clearvars -except DATA_DIR_PATH DATA_MAT_PATH; clc

% add util to path
addpath("../util")

% begin script 
fprintf('Enter MATLAB script: %s.m\n', mfilename)

% load data from .mat file
data_mat = load(DATA_MAT_PATH);
%--------------------------------------------------------------------------

% validate variables
matVarsExist(data_mat, ["A", "B", "C", "D"])

% convert state space model to transfer function
sys = ss(data_mat.A, data_mat.B, data_mat.C, data_mat.D);
G = tf(sys);

% save data to .mat file
save(DATA_MAT_PATH, "sys", "G", "-append")

%--------------------------------------------------------------------------
% end script
fprintf('Exit MATLAB script: %s.m\n', mfilename)