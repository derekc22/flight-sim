% begin script 
fprintf('Begin MATLAB script: %s.m\n', mfilename);

% clear workspace 
clearvars -except data_path; clc

% load JSON file from data path
var_json_path = fullfile(data_path, "variables.json");
data = jsondecode(fileread(var_json_path));

% load state and input dimensions
n = data.state_dim;
m = data.input_dim;

% load system matrices
[A, B, C, D] = load_ABCD(data_path, data.matrix_paths);

% save data to .mat file
save("data/data.mat", "data_path", "n", "m", "A", "B", "C", "D");

% end script
fprintf('End MATLAB script: %s.m\n', mfilename);





%% Helper functions
function [A, B, C, D] = load_ABCD(data_path, matrix_paths)
    A = readmatrix(fullfile(data_path, matrix_paths.A));
    B = readmatrix(fullfile(data_path, matrix_paths.B));
    C = readmatrix(fullfile(data_path, matrix_paths.C));
    D = readmatrix(fullfile(data_path, matrix_paths.D));
end

