% clear workspace 
clearvars -except DATA_DIR_PATH DATA_MAT_PATH; clc

% add util to path
addpath("util")

% begin script 
fprintf('Enter MATLAB script: %s.m\n', mfilename)

% load JSON from data path
var_json_path = fullfile(DATA_DIR_PATH, "variables.json");
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
[A, B, C, D] = loadABCD(DATA_DIR_PATH, json_data.matrix_paths);

% save data to .mat file
DATA_MAT_PATH = fullfile(DATA_DIR_PATH, "data.mat");
save(DATA_MAT_PATH, "DATA_DIR_PATH", "DATA_MAT_PATH", "n", "m", "A", "B", "C", "D")

%--------------------------------------------------------------------------
% end script
fprintf('Exit MATLAB script: %s.m\n', mfilename)





%% functions
function [A, B, C, D] = loadABCD(DATA_DIR_PATH, matrix_paths)
    A = readmatrix(fullfile(DATA_DIR_PATH, matrix_paths.A));
    B = readmatrix(fullfile(DATA_DIR_PATH, matrix_paths.B));
    C = readmatrix(fullfile(DATA_DIR_PATH, matrix_paths.C));
    D = readmatrix(fullfile(DATA_DIR_PATH, matrix_paths.D));
end

