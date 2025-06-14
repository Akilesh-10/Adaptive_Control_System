% Vehicle model: G(s) = 1 / (1000s + 50)
num = 1;
den = [1000 50];
plant = tf(num, den);  % Transfer function

% PID controller parameters
Kp = 800;
Ki = 40;
Kd = 100;

% Create PID controller
controller = pid(Kp, Ki, Kd);

% Closed-loop system
sys_cl = feedback(controller * plant, 1);

% Simulate step response (target speed = 10 units)
t = 0:0.1:50;
[y, t] = step(10 * sys_cl, t);

% Plot
plot(t, y);
xlabel('Time (s)');
ylabel('Speed Output');
title('Adaptive Speed Control System (MATLAB)');
grid on;
