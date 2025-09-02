% Regen with Torque Limiting and Energy Recovery
clear; clc;

%% Variable Definitions

% Vehicle & Tire Parameters
mass = 285;                  % Vehicle mass [kg]
g = 9.81;                    % Gravitational acceleration [m/s^2]
Fz = 700;
%Fz = mass * g / 4;           % Normal force on one tire (assuming 4 equal-contact wheels) [N]
R_wheel = 0.23;              % Wheel radius [m]
v_vehicle = 20;              % Constant vehicle speed [m/s]
%model 1.8-1.9
mu_max = 2.0;                % Maximum friction coefficient (road-tire interaction)

% Pacejka Tire Model Coefficients 
B = 10;                      % Stiffness factor
C = 1.9;                     % Shape factor
D = mu_max;                  % Peak friction multiplier (based on mu)
E = 0.97;                    % Curvature factor

% Simulation Timing Parameters
dt = 0.1;                  % Time step [s]
t_end = 2;                   % Total simulation duration [s]
time = 0:dt:t_end;           % Time vector for simulation

%Initial Conditions
%omega = v_vehicle / R_wheel; % Initial wheel angular velocity [rad/s]
regen_torque_cmd = -500;     % Max regenerative braking torque applied [Nm]
I_wheel = 1.2;               % Moment of inertia of the wheel [kg·m^2]

% Slip Safety Threshold
slip_limit = -0.2;           % Slip threshold below which regen is disabled (to prevent lock-up)

%Data Storage Arrays for Plotting
slip_array   = zeros(size(time)); % Slip ratio over time
Fx_array     = zeros(size(time)); % Longitudinal tire force [N]
omega_array  = zeros(size(time)); % Wheel angular speed [rad/s]
torque_array = zeros(size(time)); % Applied regen torque [Nm]
energy_recov = zeros(size(time)); % Cumulative recovered energy [J]
pacejka = zeros(size(time)); % Cumulative recovered energy [J]

% --- Energy Tracking ---
E_total = 0;                 % Initialize total energy recovered [J]

slip = [0:-0.02:-0.4];

% Simulation Loop
for i = 1:length(time)
    
    % Calculate slip ratio
    v_wheel = v_vehicle + slip(i)*v_vehicle;
    omega = v_wheel / R_wheel; % Initial wheel angular velocity [rad/s]
    %v_wheel = omega * R_wheel;
    %slip = (v_wheel - v_vehicle) / max(abs(v_vehicle), 0.1);
    %disp(slip)

    
    % Tire force using Pacejka formula?
    Fx = Fz * D * sin(C * atan(B * slip(i) - E * (B * slip(i) - atan(B * slip(i)))));
    v_vehicle = v_vehicle + ((Fx / mass)/10) ;
    
    % Apply regen torque with safety clamp
    if slip < slip_limit
        T_applied = 0;  % disengage regen if too much braking
    else
        T_applied = regen_torque_cmd;
    end

    % Wheel dynamics
    alpha = (T_applied - Fx * R_wheel) / I_wheel;
    omega = omega + alpha * dt;
    v_vehicle = v_vehicle - ((T_applied*R_wheel)/mass)/10;
    
    % Regen power recovery (motor power = torque * omega)
    P_recovered = -min(T_applied * omega, 0);  % only count braking energy
    E_total = E_total + P_recovered * dt;      % integrate power over time
    
    % Store values
    slip_array(i) = slip(i);
    Fx_array(i) = Fx;
    omega_array(i) = omega;
    torque_array(i) = T_applied;
    energy_recov(i) = E_total;
    pacejka(i) = Fx;
    v_vehiclearray(i) = v_vehicle;
    
end

% Plot Results
figure;

% Pacejka
subplot(6,1,5);
plot(slip_array, Fx_array);
ylabel('Tire Force'); title('Pacejka');
grid on;

% 1. Slip Ratio vs Time
subplot(6,1,1);
plot(time, slip_array);
ylabel('Slip Ratio'); title('1. Slip Ratio');
grid on;

% 2. Tire Force vs Time
subplot(6,1,2);
plot(time, Fx_array);
ylabel('Fx [N]'); title('2. Longitudinal Tire Force');
grid on;

% 3. Wheel Speed vs Time
subplot(6,1,3);
plot(time, omega_array * R_wheel);
ylabel('Wheel Speed [m/s]'); title('3. Wheel Speed');
grid on;

% 4. Recovered Energy vs Time
subplot(6,1,4);
plot(time, energy_recov);
ylabel('Energy [J]'); xlabel('Time [s]');
title('4. Cumulative Regen Energy Recovered');
grid on;

% 5. Vehicle Speed
subplot(6,1,6);
plot(time, v_vehiclearray);
ylabel('Vehicle Speed'); xlabel('Time [s]');
title('Vehicle Speed');
grid on;

