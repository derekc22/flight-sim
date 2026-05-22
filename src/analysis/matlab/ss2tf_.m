% clear workspace 
clearvars -except data_dir_path data_mat_path plot_dir_path; clc

% add util to path
addpath("util")

% begin script 
fprintf('Enter MATLAB script: %s.m\n', mfilename)

% load data from .mat file
data_mat = load(data_mat_path);
%--------------------------------------------------------------------------

% validate variables
matVarsExist(data_mat, ["A", "B", "C", "D"])

% convert state space model to transfer function
sys = ss(data_mat.A, data_mat.B, data_mat.C, data_mat.D);
G = tf(sys);

% save data to .mat file
save(data_mat_path, "sys", "G", "-append")

%--------------------------------------------------------------------------
% end script
fprintf('Exit MATLAB script: %s.m\n', mfilename)