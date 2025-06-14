clc; clear; close all;

% --- Simulation Parameters ---
t = 0:0.1:100;             % Time vector
dt = t(2) - t(1);
r = 5 * (t < 30) + 10 * (t >= 30 & t < 70) + 7 * (t >= 70);  % Reference profile

% --- Plant: Vehicle Transfer Function 1 / (ms + b) ---
m = 1000;  % Mass
b = 50;    % Damping
plant = tf(1, [m b]);

% --- Disturbance Model ---
disturbance = 0.1 * randn(size(t));  % Simulate small noise/disturbance

% --- Initialize ---
y = zeros(size(t));           % Output (Speed)
e = zeros(size(t));           % Error
u = zeros(size(t));           % Control effort
Kp = 800; Ki = 40; Kd = 100;  % Initial PID gains
I = 0; D = 0; prev_error = 0;

for i = 2:length(t)
    % --- Error Calculation ---
    e(i) = r(i) - y(i-1);
    
    % --- Adaptive PID Logic ---
    if abs(e(i)) > 3
        Kp = 1000; Ki = 50; Kd = 150;
    else
        Kp = 800; Ki = 40; Kd = 100;
    end

    % --- PID Controller ---
    I = I + e(i) * dt;                        % Integral term
    D = (e(i) - prev_error) / dt;            % Derivative term
    u(i) = Kp*e(i) + Ki*I + Kd*D;             % Control effort
    u(i) = max(min(u(i), 100), -100);        % Actuator saturation
    prev_error = e(i);

    % --- Plant Simulation (Discrete Euler Integration) ---
    dy = (1/m)*(u(i) - b*y(i-1));             % dy/dt = (u - b*y)/m
    y(i) = y(i-1) + dy*dt + disturbance(i);   % Add disturbance
end

%---

% --- Visualization ---
figure;
subplot(3,1,1);
plot(t, r, 'r--', t, y, 'b'); grid on;
legend('Setpoint', 'Output'); title('Speed Tracking');

subplot(3,1,2);
plot(t, e); grid on;
title('Tracking Error');

subplot(3,1,3);
plot(t, u); grid on;
title('Control Effort (u)');
xlabel('Time (s)');
