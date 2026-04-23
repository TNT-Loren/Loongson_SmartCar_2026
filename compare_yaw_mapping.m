clear;
clc;
close all;

% Input range from the normalized deviation output.
x = linspace(-1.0, 1.0, 1000);

% Old linear mapping.
k_dev_to_yaw = 20.0;
y_linear = k_dev_to_yaw * x;

% Piecewise turn mapping copied from map_dev_to_angle().
yaw_limit = 55.0;
y_piecewise = arrayfun(@map_dev_to_angle, x);
u_detail = linspace(0.0, 1.0, 1000);
yaw_detail = arrayfun(@map_dev_to_angle, u_detail);
u_knots = [0.00, 0.03, 0.125, 0.30, 0.35, 0.40, 0.50, 0.60, 1.00];
yaw_knots = [0.0, 0.0, 5.0, 15.0, 25.0, 30.0, 45.0, 55.0, 55.0];

figure('Color', 'w', 'Name', 'Yaw Mapping Compare');
plot(x, y_linear, 'LineWidth', 2.5, 'Color', [0.85, 0.33, 0.10]);
hold on;
plot(x, y_piecewise, 'LineWidth', 2.5, 'Color', [0.00, 0.45, 0.74]);
grid on;
box on;
xlabel('Deviation');
ylabel('Vision Delta Yaw (deg)');
title('Linear vs Right-Angle Turn Yaw Mapping');
legend( ...
    sprintf('Linear: y = %.1f * x', k_dev_to_yaw), ...
    sprintf('Piecewise map\\_dev\\_to\\_angle(), sat to +/-%.1f deg', yaw_limit), ...
    'Location', 'northwest');

figure('Color', 'w', 'Name', 'Right-Angle Turn Mapping Detail');
plot(u_detail, yaw_detail, 'LineWidth', 2.5, 'Color', [0.49, 0.18, 0.56]);
hold on;
plot(u_knots, yaw_knots, 'o', 'MarkerSize', 7, 'LineWidth', 1.5, 'Color', [0.85, 0.33, 0.10]);
grid on;
box on;
xlabel('u = |Deviation|');
ylabel('Yaw Output (deg)');
title('Right-Angle Turn Curve');
legend( ...
    'map\_dev\_to\_angle()', ...
    'Target points', ...
    'Location', 'northwest');

disp('Parameters used in this script:');
fprintf('k_dev_to_yaw = %.2f\n', k_dev_to_yaw);
fprintf('yaw_limit = %.2f deg\n', yaw_limit);
disp('Target points (u, yaw_deg):');
disp([u_knots(:), yaw_knots(:)]);

function angle = map_dev_to_angle(deviation)
    x = abs(deviation);
    y = 0.0;

    if x <= 0.03
        y = 0.0;
    elseif x <= 0.125
        t = (x - 0.03) / 0.095;
        y = 5.0 * t * t;
    elseif x <= 0.3
        u = (x - 0.125) / 0.175;
        y = 5.0 + 10.0 * smoothstep(u);
    elseif x <= 0.35
        v = (x - 0.3) / 0.05;
        y = 15.0 + 10.0 * smoothstep(v);
    elseif x <= 0.4
        w = (x - 0.35) / 0.05;
        y = 25.0 + 5.0 * smoothstep(w);
    elseif x <= 0.5
        q = (x - 0.4) / 0.1;
        y = 30.0 + 15.0 * smoothstep(q);
    elseif x <= 0.6
        r = (x - 0.5) / 0.1;
        y = 45.0 + 10.0 * smoothstep(r);
    else
        y = 55.0;
    end

    angle = sign(deviation) * y;
end

function y = smoothstep(t)
    y = t .* t .* (3.0 - 2.0 .* t);
end
