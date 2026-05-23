% clear workspace 
clearvars -except DATA_DIR_PATH DATA_MAT_PATH PLOT_DIR_PATH CONFIG_PATH; clc

% add util to path
addpath("util")

% begin script 
fprintf('Enter MATLAB script: %s.m\n', mfilename)

% load data from .mat file
data_mat = load(DATA_MAT_PATH);
%--------------------------------------------------------------------------

% validate variables
matVarsExist(data_mat, ["G"])

% make plot directory as it may not exist
mkdir(PLOT_DIR_PATH)

[nOutputs, nInputs] = size(data_mat.G);
t = 0:0.01:10;

% step response
for inputIdx = 1:nInputs
    U = zeros(length(t), nInputs);
    U(:, inputIdx) = 1;

    y = lsim(data_mat.G, U, t);

    figure();

    for outputIdx = 1:nOutputs
        subplot(4, 2, outputIdx);
        plot(t, y(:, outputIdx), 'LineWidth', 2);
        grid on;
        title(sprintf('Output %d', outputIdx));
        xlabel('Time [s]');
        ylabel(sprintf('y_%d', outputIdx));
    end

    sgtitle(sprintf('Step Response, Input %d', inputIdx));

    fname = sprintf("step_response_input_%d.pdf", inputIdx);
    print(gcf, char(fullfile(PLOT_DIR_PATH, fname)), "-dpdf", "-painters", "-bestfit")
    close(gcf)
end



% ramp response
for inputIdx = 1:nInputs
    U = zeros(length(t), nInputs);
    U(:, inputIdx) = t;

    y = lsim(data_mat.G, U, t);

    figure();

    for outputIdx = 1:nOutputs
        subplot(4, 2, outputIdx);
        plot(t, y(:, outputIdx), 'LineWidth', 2);
        grid on;
        title(sprintf('Output %d', outputIdx));
        xlabel('Time [s]');
        ylabel(sprintf('y_%d', outputIdx));
    end

    sgtitle(sprintf('Ramp Response, Input %d', inputIdx));

    fname = sprintf("ramp_response_input_%d.pdf", inputIdx);
    print(gcf, char(fullfile(PLOT_DIR_PATH, fname)), "-dpdf", "-painters", "-bestfit")
    close(gcf)
end



% sine response
for inputIdx = 1:nInputs

    U = zeros(length(t), nInputs);
    U(:, inputIdx) = sin(t);

    y = lsim(data_mat.G, U, t);

    figure();

    for outputIdx = 1:nOutputs
        subplot(4, 2, outputIdx);
        plot(t, y(:, outputIdx), 'LineWidth', 2);
        grid on;
        title(sprintf('Output %d', outputIdx));
        xlabel('Time [s]');
        ylabel(sprintf('y_%d', outputIdx));
    end

    sgtitle(sprintf('Sine Response, Input %d', inputIdx));

    fname = sprintf("sine_response_input_%d.pdf", inputIdx);
    print(gcf, char(fullfile(PLOT_DIR_PATH, fname)), "-dpdf", "-painters", "-bestfit")
    close(gcf)
end