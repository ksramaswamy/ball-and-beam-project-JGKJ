function plot_states(ts, xs, ref_ps, ref_vs, theta_ds, xms)

if nargin < 5
    theta_ds = []; 
end
if nargin < 6
    xms = []; 
end

fig = figure();
% Set up font size.
set(fig, 'DefaultAxesFontSize', 16);
% Set up font name
set(fig, 'DefaultTextFontName', 'Times New Roman');
% Set up interpreter
set(fig, 'DefaultTextInterpreter', 'latex');

subplot(4, 1, 1);
plot(ts, 100 * xs(1, :), 'LineWidth', 1.5);
hold on;
plot(ts, 100 * ref_ps, '-.', 'LineWidth', 1.5);
if ~isempty(xms)
    plot(ts, 100 * xms(1,:),'-.k', 'LineWidth', 1.5);
end
legend('True Trajectory','Reference','Estimate')

ylabel('$z$ [cm]', 'Interpreter', 'latex');
grid on;
title('State History');


subplot(4, 1, 2);
plot(ts, 100 * xs(2, :), 'LineWidth', 1.5);
hold on;
plot(ts, 100 * ref_vs, '-.', 'LineWidth', 1.5);
grid on;
ylabel('$\dot{z}$ [cm / s]', 'Interpreter', 'latex');
if ~isempty(xms)
    plot(ts, 100 * xms(2,:), '-.k', 'LineWidth', 1.5);
end

subplot(4, 1, 3);
plot(ts, 180 * xs(3, :) / pi, 'LineWidth', 1.5);
ylabel('$\theta$ [deg]', 'Interpreter', 'latex');
if ~isempty(xms)
    hold on;
    plot(ts, 180 * xms(3,:) / pi, 'k-.', 'LineWidth', 1.5);
    grid on;
end

subplot(4, 1, 4);
plot(ts, 180 * xs(4, :) / pi, 'LineWidth', 1.5);
ylabel('$\dot{\theta}$ [deg/s]', 'Interpreter', 'latex');
xlabel('$t$ [sec]', 'Interpreter', 'latex');
grid on;
hold on;
if ~isempty(xms)
    plot(ts, 180 * xms(4,:) / pi, '-.k', 'LineWidth', 1.5);
end
axis([0 90 -20 20])
end