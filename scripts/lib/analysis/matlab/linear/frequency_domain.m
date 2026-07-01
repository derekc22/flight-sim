PROJ_PATH="/Users/derekchibuzor/Documents/VSCodeProjects/flight-sim";
ANALYSIS_TYPE="frequency_domain";
rel_path="analysis/linear/frequency_domain/default.json";
OUT_DIR="2026Jun21_23-38-39";

% clear workspace 
% clearvars -except DATA_DIR_PATH DATA_MAT_PATH PLOT_DIR_PATH REPORT_DIR_PATH CONFIG_PATH; clc

% add util to path
addpath("../util")

% begin script 
fprintf('Enter MATLAB script: %s.m\n', mfilename)

% load data from .mat file
data_mat = load(DATA_MAT_PATH);

% load JSON from config path
config_path = fullfile(CONFIG_PATH);
json_config = jsondecode(fileread(config_path));
%--------------------------------------------------------------------------

% validate variables
matVarsExist(data_mat, [ ...
    "sys" ...
])

% validate fields
jsonFieldsExist(json_config, [ ...
    "w_min_power", ...
    "w_max_power", ...
    "num_points", ...
    "bode", ...
    "nyquist", ...
    "nichols", ...
    "singular_value", ...
    "frequency_response" ...
])

[n_outputs, n_inputs] = size(data_mat.sys);
w = logspace(json_config.w_min_power, json_config.w_max_power, json_config.num_points);

% bode response
if json_config.bode
    plotBodeResponse( ...
        data_mat.sys, ...
        w, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH ...
    );
end

% nyquist response
if json_config.nyquist
    plotNyquistResponse( ...
        data_mat.sys, ...
        w, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH ...
    );
end

% nichols response
if json_config.nichols
    plotNicholsResponse( ...
        data_mat.sys, ...
        w, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH ...
    );
end

% singular value response
if json_config.singular_value
    plotSingularValueResponse( ...
        data_mat.sys, ...
        w, ...
        PLOT_DIR_PATH ...
    );
end

%--------------------------------------------------------------------------
% end script
fprintf('Exit MATLAB script: %s.m\n', mfilename)





%% functions
function plotBodeResponse(G, w, n_outputs, n_inputs, plot_dir_path)
    for input_idx = 1:n_inputs
        for output_idx = 1:n_outputs
            figure();
    
            bode(G(output_idx, input_idx), w);
            grid on;
            title(sprintf('bode response: input %d, output %d', input_idx, output_idx));
    
            fname = sprintf("bode_response_input_%d_output_%d.pdf", input_idx, output_idx);
            save_path = fullfile(plot_dir_path, fname);
            print(gcf, char(save_path), "-dpdf", "-vector", "-bestfit")
            fprintf("File saved successfully to %s\n", char(save_path))
    
            close(gcf)
        end
    end
end

function plotNyquistResponse(G, w, n_outputs, n_inputs, plot_dir_path)
    for input_idx = 1:n_inputs
        for output_idx = 1:n_outputs
            figure();
    
            nyquist(G(output_idx, input_idx), w);
            grid on;
            title(sprintf('nyquist response: input %d, output %d', input_idx, output_idx));
    
            fname = sprintf("nyquist_response_input_%d_output_%d.pdf", input_idx, output_idx);
            save_path = fullfile(plot_dir_path, fname);
            print(gcf, char(save_path), "-dpdf", "-vector", "-bestfit")
            fprintf("File saved successfully to %s\n", char(save_path))
    
            close(gcf)
        end
    end
end

function plotNicholsResponse(G, w, n_outputs, n_inputs, plot_dir_path)
    for input_idx = 1:n_inputs
        for output_idx = 1:n_outputs
            figure();
    
            nichols(G(output_idx, input_idx), w);
            grid on;
            title(sprintf('nichols response: input %d, output %d', input_idx, output_idx));
    
            fname = sprintf("nichols_response_input_%d_output_%d.pdf", input_idx, output_idx);
            save_path = fullfile(plot_dir_path, fname);
            print(gcf, char(save_path), "-dpdf", "-vector", "-bestfit")
            fprintf("File saved successfully to %s\n", char(save_path))
    
            close(gcf)
        end
    end
end

function plotSingularValueResponse(G, w, plot_dir_path)
    figure();
    
    sigma(G, w);
    grid on;
    title('singular value response');
    
    fname = "singular_value_response.pdf";
    save_path = fullfile(plot_dir_path, fname);
    print(gcf, char(save_path), "-dpdf", "-vector", "-bestfit")
    fprintf("File saved successfully to %s\n", char(save_path))
    
    close(gcf)
end