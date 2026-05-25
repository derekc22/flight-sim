% clear workspace 
clearvars -except DATA_DIR_PATH DATA_MAT_PATH PLOT_DIR_PATH CONFIG_PATH; clc

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
    "dt", ...
    "G" ...
])

% validate fields
jsonFieldsExist(json_config, [ ...
    "time_sec", ...
    "step", ...
    "impulse", ...
    "ramp", ...
    "sine" ...
])

% make plot directory as it may not exist
mkdir(PLOT_DIR_PATH)

[n_outputs, n_inputs] = size(data_mat.G);
t = 0:data_mat.dt:json_config.time_sec;

% open txt file
info_fid = fopen(fullfile(DATA_DIR_PATH, "lsiminfo.txt"), "w");

% step response
if json_config.step
    simulateAndPlotResponse( ...
        data_mat.G, ...
        data_mat.dt, ...
        t, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH, ...
        info_fid, ...
        "step_response", ...
        @(t, dt) ones(length(t), 1) ...
    );
end

% impulse response
if json_config.impulse
    simulateAndPlotResponse( ...
        data_mat.G, ...
        data_mat.dt, ...
        t, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH, ...
        info_fid, ...
        "impulse_response", ...
        @makeImpulseInput ...
    );
end

% ramp response
if json_config.ramp
    simulateAndPlotResponse( ...
        data_mat.G, ...
        data_mat.dt, ...
        t, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH, ...
        info_fid, ...
        "ramp_response", ...
        @(t, dt) t ...
    );
end

% sine response
if json_config.sine
    simulateAndPlotResponse( ...
        data_mat.G, ...
        data_mat.dt, ...
        t, ...
        n_outputs, ...
        n_inputs, ...
        PLOT_DIR_PATH, ...
        info_fid, ...
        "sine_response", ...
        @(t, dt) sin(t) ...
    );
end

fclose(info_fid);

%--------------------------------------------------------------------------
% end script
fprintf('Exit MATLAB script: %s.m\n', mfilename)





%% functions
function simulateAndPlotResponse(G, dt, t, n_outputs, n_inputs, plot_dir_path, info_fid, file_prefix, make_input_signal)
    response_title = strrep(file_prefix, "_", " ");

    for input_idx = 1:n_inputs
        u = zeros(length(t), n_inputs);
        u(:, input_idx) = make_input_signal(t(:), dt);

        y = lsim(G, u, t);

        figure();

        plotOutputs(t, y, n_outputs);

        sgtitle(sprintf('%s, input %d', response_title, input_idx));

        fname = sprintf("%s_%d.pdf", file_prefix, input_idx);
        print(gcf, char(fullfile(plot_dir_path, fname)), "-dpdf", "-vector", "-bestfit")

        close(gcf)

        info = lsiminfo(y, t);
        writeLsimInfo(info_fid, info, file_prefix, input_idx)
    end
end

function plotOutputs(t, y, n_outputs)
    for output_idx = 1:n_outputs
        subplot(4, 2, output_idx);
        plot(t, y(:, output_idx), 'LineWidth', 2);
        grid on;
        title(sprintf('output %d', output_idx));
        xlabel('time [s]');
        ylabel(sprintf('y_%d', output_idx));
    end
end

function writeLsimInfo(info_fid, info, file_prefix, input_idx)
    response_title = strrep(file_prefix, "_", " ");
    fields = fieldnames(info);

    fprintf(info_fid, "%s, input %d\n", response_title, input_idx);
    fprintf(info_fid, "%s\n", repmat('-', 1, 60));

    for output_idx = 1:length(info)
        fprintf(info_fid, "output %d\n", output_idx);

        for field_idx = 1:length(fields)
            field_name = fields{field_idx};
            field_value = info(output_idx).(field_name);

            fprintf(info_fid, "  %s: %.10g\n", field_name, field_value);
        end

        fprintf(info_fid, "\n");
    end

    fprintf(info_fid, "\n");
end

function u = makeImpulseInput(t, dt)
    u = zeros(length(t), 1);
    u(1) = 1 / dt;
end