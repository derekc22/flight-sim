% clear workspace 
clearvars -except data_dir_path data_mat_path data_dir_path clc

% add util to path
addpath("util")

% begin script 
fprintf('Enter MATLAB script: %s.m\n', mfilename)

% load JSON from data path
var_json_path = fullfile(data_dir_path, "variables.json");
json_data = jsondecode(fileread(var_json_path));
%--------------------------------------------------------------------------

% validate fields
jsonFieldsExist(json_data, [ ...
    "state_dim", ...
    "input_dim", ...
    "matrix_paths.A", ...
    "matrix_paths.B", ...
    "matrix_paths.C", ...
    "matrix_paths.D"
])

% load state and input dimensions
n = json_data.state_dim;
m = json_data.input_dim;

% load system matrices
[A, B, C, D] = loadABCD(data_dir_path, json_data.matrix_paths);

% save data to .mat file
data_mat_path = fullfile(data_dir_path, "data.mat");
save(data_mat_path, "data_dir_path", "data_mat_path", "n", "m", "A", "B", "C", "D")

%--------------------------------------------------------------------------
% end script
fprintf('Exit MATLAB script: %s.m\n', mfilename)





%% Functions
function [A, B, C, D] = loadABCD(data_dir_path, matrix_paths)
    A = readmatrix(fullfile(data_dir_path, matrix_paths.A));
    B = readmatrix(fullfile(data_dir_path, matrix_paths.B));
    C = readmatrix(fullfile(data_dir_path, matrix_paths.C));
    D = readmatrix(fullfile(data_dir_path, matrix_paths.D));
end

