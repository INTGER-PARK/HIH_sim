function HandinHand_plot(csv_path)
%HANDINHAND_PLOT Plot independent Leader/Follower and Follower admittance data.
%   HandinHand_plot() finds the newest src/hih_logging/recordings/run_*/ CSV.
%   HandinHand_plot('/absolute/path/to/all_topics.csv') uses a chosen run.
%   CSV columns from hih_logging: t_sec, topic_name, and
%   flattened ROS message fields such as force_2, pos_2 and pos_cmd_2.

if nargin < 1 || strlength(string(csv_path)) == 0
    script_dir = fileparts(mfilename('fullpath'));
    matches = dir(fullfile(script_dir, '..', 'src', 'hih_logging', ...
        'recordings', 'run_*', 'all_topics.csv'));
    assert(~isempty(matches), 'No recording found; pass a CSV path explicitly.');
    [~, newest] = max([matches.datenum]);
    csv_path = fullfile(matches(newest).folder, matches(newest).name);
end

assert(isfile(csv_path), 'Missing CSV: %s', csv_path);
data = readtable(csv_path, 'VariableNamingRule', 'preserve');
assert(all(ismember({'t_sec', 'topic_name'}, data.Properties.VariableNames)), ...
    'Expected t_sec and topic_name columns.');
t0 = min(data.t_sec);
fprintf('HandinHand data: %s (%d messages)\n', csv_path, height(data));

fstate = rows(data, '/follower/minitrone/state');
lstate = rows(data, '/leader/minitrone/state');
fcmd = rows(data, '/follower/minitrone/cmd');
lcmd = rows(data, '/leader/minitrone/cmd');
fadm = rows(data, '/follower/minitrone/cmd_admittance');
fcwrench = rows(data, '/follower/minitrone/admittance/contact_wrench');
fext = rows(data, '/follower/minitrone/external_wrench_hat_second_order');
lext = rows(data, '/leader/minitrone/external_wrench_hat_second_order');
fblend = rows(data, '/follower/minitrone/admittance/blend');
fcontact = rows(data, '/follower/minitrone/cop_real');
lcontact = rows(data, '/leader/minitrone/cop_real');

% Compare measured positions with task and admittance references.
figure('Name', 'Drone positions', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
axes_names = {'X', 'Y', 'Z'};
for k = 0:2
    nexttile; hold on; grid on;
    draw(fstate, t0, sprintf('pos_%d', k), 'Follower actual', '-');
    draw(fcmd, t0, sprintf('pos_cmd_%d', k), 'Follower base command', '--');
    draw(lstate, t0, sprintf('pos_%d', k), 'Leader actual', '-.');
    draw(lcmd, t0, sprintf('pos_cmd_%d', k), 'Leader command', ':');
    ylabel(sprintf('%s [m]', axes_names{k + 1}));
    if k == 0, legend('Location', 'best'); end
end
xlabel('time since recording start [s]');

figure('Name', 'Contact and admittance', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile; hold on; grid on;
draw(fcontact, t0, 'normal_force', 'Follower plate', '-');
draw(lcontact, t0, 'normal_force', 'Leader plate', '-.');
fdes = rows(data, '/follower/minitrone/admittance_des_force');
draw(fdes, t0, 'data', 'Follower force target', '--');
ylabel('normal force [N]'); legend('Location', 'best');
title('Local contact and preload');

nexttile; hold on; grid on;
draw(fext, t0, 'force_0', 'Follower MoB F_x', '-');
draw(lext, t0, 'force_0', 'Leader MoB F_x', '--');
ylabel('force [N]'); legend('Location', 'best');
title('Normal-axis external force estimates');

nexttile; hold on; grid on;
draw(fblend, t0, 'data', 'admittance blend', '-');
xlabel('time since recording start [s]');
ylabel('blend'); legend('Location', 'best');

% Mirror the source script's six-axis wrench overview for both vehicles.
plot_wrenches(data, t0, 'follower');
plot_wrenches(data, t0, 'leader');
if ~isempty(fcwrench)
    figure('Name', 'Follower CoM to contact wrench', 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact');
    nexttile; hold on; grid on;
    draw(fext, t0, 'force_2', 'MoB Body F_z', '-');
    draw(fcwrench, t0, 'force_2', 'Contact C F_z', '--');
    ylabel('force [N]'); legend('Location', 'best');
    nexttile; hold on; grid on;
    draw(fext, t0, 'moment_2', 'MoB CoM moment Z', '-');
    draw(fcwrench, t0, 'moment_2', 'Contact center moment Z', '--');
    xlabel('time since recording start [s]'); ylabel('moment [N m]');
    legend('Location', 'best');
end
plot_inputs(data, t0, 'follower');
plot_inputs(data, t0, 'leader');
end

function plot_wrenches(all_data, t0, drone)
    prefix = '/' + string(drone) + '/minitrone/';
    commanded = rows(all_data, prefix + 'wrench_cmd');
    actual = rows(all_data, prefix + 'actuation_wrench_body');
    estimated = rows(all_data, prefix + 'external_wrench_hat_second_order');
    figure('Name', char(string(drone) + ' six-axis wrench'), 'Color', 'w');
    tiledlayout(3, 2, 'TileSpacing', 'compact');
    for k = 0:2
        nexttile; hold on; grid on;
        field = sprintf('force_%d', k);
        draw(commanded, t0, field, 'command', '-');
        draw(actual, t0, field, 'actual actuation', '-.');
        draw(estimated, t0, field, 'external estimate', ':');
        ylabel(sprintf('F_%s [N]', char('x' + k)));
        if k == 0, legend('Location', 'best'); end
        nexttile; hold on; grid on;
        field = sprintf('moment_%d', k);
        draw(commanded, t0, field, 'command', '-');
        draw(actual, t0, field, 'actual actuation', '-.');
        draw(estimated, t0, field, 'external estimate', ':');
        ylabel(sprintf('M_%s [Nm]', char('x' + k)));
    end
    xlabel('time since recording start [s]');
end

function plot_inputs(all_data, t0, drone)
    actuator = rows(all_data, '/' + string(drone) + '/minitrone/input');
    if isempty(actuator), return; end
    figure('Name', char(string(drone) + ' motor and servo commands'), 'Color', 'w');
    tiledlayout(4, 2, 'TileSpacing', 'compact');
    for k = 0:3
        nexttile; hold on; grid on;
        draw(actuator, t0, sprintf('u_%d', k), 'motor command', '-');
        ylabel(sprintf('motor %d', k + 1));
        nexttile; hold on; grid on;
        draw(actuator, t0, sprintf('u_%d', k + 4), 'servo command', '-');
        ylabel(sprintf('servo %d', k + 1));
    end
    xlabel('time since recording start [s]');
end

function out = rows(table_data, topic)
    out = table_data(strcmp(string(table_data.topic_name), topic), :);
end

function draw(topic_rows, t0, field, label, style)
    if isempty(topic_rows) || ~ismember(field, topic_rows.Properties.VariableNames)
        return;
    end
    values = topic_rows.(field);
    if iscell(values) || isstring(values), values = str2double(values); end
    plot(topic_rows.t_sec - t0, values, style, ...
        'LineWidth', 1.2, 'DisplayName', label);
end
