% plot_controlled_oscillator.m
% Visualize the controlled mass-spring-damper system performance

clear all; close all; clc;

%% Load data
filename = 'controlled_oscillator.csv';

if ~isfile(filename)
    error('File %s not found. Please run the C++ controller program first.', filename);
end

data = readtable(filename);

% Extract data
time = data.Time_s_;
position = data.Position_m_;
velocity = data.Velocity_m_s_;
energy = data.Energy_J_;
control_force = data.ControlForce_N_;
desired_position = data.DesiredPosition_m_;
desired_velocity = data.DesiredVelocity_m_s_;
position_error = data.PositionError_m_;

%% Figure 1: Main Control Performance
figure('Position', [100, 100, 1200, 800], 'Name', 'Control System Performance');

% Position tracking
subplot(3,2,1);
plot(time, desired_position, 'r--', 'LineWidth', 1.5);
hold on;
plot(time, position, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Position Tracking');
legend('Desired', 'Actual', 'Location', 'best');

% Velocity tracking
subplot(3,2,2);
plot(time, desired_velocity, 'r--', 'LineWidth', 1.5);
hold on;
plot(time, velocity, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Tracking');
legend('Desired', 'Actual', 'Location', 'best');

% Control force
subplot(3,2,3);
plot(time, control_force, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Control Input');
ylim([min(control_force)*1.1, max(control_force)*1.1]);

% Tracking error
subplot(3,2,4);
plot(time, position_error, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (m)');
title('Position Tracking Error');
yline(0, 'k--');

% Phase portrait
subplot(3,2,5);
plot(position, velocity, 'b-', 'LineWidth', 1.5);
hold on;
plot(position(1), velocity(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(position(end), velocity(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
xlabel('Position (m)');
ylabel('Velocity (m/s)');
title('Phase Portrait');
legend('Trajectory', 'Start', 'End', 'Location', 'best');
axis equal;

% Energy
subplot(3,2,6);
plot(time, energy, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Energy (J)');
title('System Energy');

sgtitle('Controlled Mass-Spring-Damper System Performance');

%% Figure 2: Detailed Analysis
figure('Position', [150, 150, 1000, 600], 'Name', 'Control Analysis');

% Error analysis
subplot(2,2,1);
plot(time, abs(position_error), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('|Error| (m)');
title('Absolute Position Error');

% Control effort vs error
subplot(2,2,2);
scatter(position_error, control_force, 2, 'b');
grid on;
xlabel('Position Error (m)');
ylabel('Control Force (N)');
title('Control Force vs Error');

% Settling time analysis
subplot(2,2,3);
steady_state = desired_position(end);
tolerance = 0.02 * abs(steady_state); % 2% settling criterion
if steady_state ~= 0
    normalized_pos = (position - steady_state) / steady_state;
    plot(time, normalized_pos * 100, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(2, 'r--', '2% Band');
    yline(-2, 'r--');
    yline(0, 'k-');
    grid on;
    xlabel('Time (s)');
    ylabel('Error (%)');
    title('Settling Time Analysis');
    ylim([-20, 20]);
else
    plot(time, position, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position Response');
end

% Statistics
subplot(2,2,4);
text(0.1, 0.9, 'Performance Metrics:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Max Error: %.4f m', max(abs(position_error))), 'FontSize', 11);
text(0.1, 0.60, sprintf('RMS Error: %.4f m', rms(position_error)), 'FontSize', 11);
text(0.1, 0.45, sprintf('Max Control: %.2f N', max(abs(control_force))), 'FontSize', 11);
text(0.1, 0.30, sprintf('Final Error: %.4f m', position_error(end)), 'FontSize', 11);

% Calculate settling time (2% criterion)
settling_indices = find(abs(position_error) < tolerance);
if ~isempty(settling_indices)
    settling_time = time(settling_indices(1));
    text(0.1, 0.15, sprintf('Settling Time: %.2f s', settling_time), 'FontSize', 11);
end

axis off;
title('Performance Summary');

sgtitle('Control System Analysis');

%% Figure 3: Control Components (if PID)
figure('Position', [200, 200, 800, 600], 'Name', 'Control Signal Analysis');

% Estimate P, I, D components from control signal
% This is approximate since we don't have the internal controller states
dt = time(2) - time(1);

% Proportional component (roughly)
P_component = position_error * (max(abs(control_force))/max(abs(position_error)));

% Integral component (roughly)
integral_error = cumtrapz(time, position_error);
I_component = integral_error * 0.1; % Scaled for visibility

% Derivative component (roughly)
derivative_error = [0; diff(position_error)/dt];
D_component = derivative_error * 0.5; % Scaled for visibility

subplot(2,1,1);
plot(time, control_force, 'k-', 'LineWidth', 2);
hold on;
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Total Control Force');
legend('Control Input');

subplot(2,1,2);
plot(time, position_error, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, integral_error/max(abs(integral_error))*max(abs(position_error)), 'b-', 'LineWidth', 1.5);
plot(time, derivative_error/max(abs(derivative_error))*max(abs(position_error)), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Normalized Signals');
title('Error Components');
legend('Error', 'Integral (scaled)', 'Derivative (scaled)', 'Location', 'best');

%% Save figures
saveas(figure(1), 'control_performance.png');
saveas(figure(2), 'control_analysis.png');
saveas(figure(3), 'control_components.png');

fprintf('Analysis complete! Figures saved.\n');

%% Display summary
fprintf('\n=== Control Performance Summary ===\n');
fprintf('Maximum position error: %.4f m\n', max(abs(position_error)));
fprintf('RMS position error: %.4f m\n', rms(position_error));
fprintf('Maximum control force: %.2f N\n', max(abs(control_force)));
fprintf('Average control power: %.2f W\n', mean(abs(control_force .* velocity)));
fprintf('Final steady-state error: %.4f m\n', position_error(end));

% Check for oscillation
osc_threshold = 0.01;
zero_crossings = sum(abs(diff(sign(position_error))) > 0);
if zero_crossings > 10
    fprintf('System shows oscillatory behavior (%d oscillations)\n', zero_crossings/2);
else
    fprintf('System shows smooth response\n');
end