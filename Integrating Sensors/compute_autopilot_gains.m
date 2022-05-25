addpath('../chap5')
load transfer_function_coef
addpath('../parameters')
simulation_parameters  
% You may choose different ways to include the above files/parameters

% AP stands for autopilot
AP.gravity = 9.8; % gravity constant
AP.sigma = 0.05;  % low pass filter gain for derivative
AP.Va0 = TF.Va_trim;
AP.Ts = SIM.ts_control;
AP.aileron_max = 45;
AP.phi_max = 15;
AP.up_limit = 45*pi/180;
AP.low_limit = -45*pi/180;

%----------roll loop-------------
wn_roll = sqrt((abs(TF.a_phi2)*AP.aileron_max)/AP.phi_max);
% wn_roll = 7;
zeta_roll = 0.707;
AP.roll_kp = ((wn_roll)^2)/TF.a_phi2;
AP.roll_kd = ((2*zeta_roll*wn_roll) - TF.a_phi1)/TF.a_phi2;

%----------course loop-------------
W_chi = 450;                 % Seperation between wn_course and wn_roll
wn_course = wn_roll/W_chi;
zeta_course = 2.1;
AP.course_kp = (2*wn_course*zeta_course*AP.Va0)/AP.gravity;
AP.course_ki = ((wn_course)^2*AP.Va0)/AP.gravity;

%----------yaw damper-------------
AP.yaw_damper_tau_r = 0.7; 
AP.yaw_damper_kp = 1.5; 
AP.p_wo = 1/AP.yaw_damper_tau_r;
AP.yaw_kr = AP.yaw_damper_kp;

%----------pitch loop-------------
wn_pitch = 20.0;
zeta_pitch = 0.707;
AP.pitch_kp = ((wn_pitch)^2 - TF.a_theta2)/TF.a_theta3;
AP.pitch_kd = ((2*zeta_pitch*wn_pitch) - TF.a_theta1)/TF.a_theta3;
K_theta_DC = (AP.pitch_kp*TF.a_theta3)/(TF.a_theta2 + (AP.pitch_kp*TF.a_theta3));   

%----------altitude loop-------------
W_h = 30.0;                 % Seperation between wn_altitude and wn_pitch 
wn_altitude = wn_pitch/W_h;
zeta_altitude = 1.0;
AP.altitude_kp = (2*zeta_altitude*wn_altitude)/(K_theta_DC*AP.Va0);
AP.altitude_ki = ((wn_altitude)^2)/(K_theta_DC*AP.Va0);
AP.altitude_zone = 10.0;  % moving saturation limit around current altitude

%---------airspeed hold using throttle---------------
wn_airspeed_throttle = 0.0001;
zeta_airspeed_throttle = 0.1;
AP.airspeed_throttle_kp = (2*wn_airspeed_throttle*zeta_airspeed_throttle - TF.a_V1)/TF.a_V2;
AP.airspeed_throttle_ki = ((wn_airspeed_throttle)^2)/TF.a_V2;
