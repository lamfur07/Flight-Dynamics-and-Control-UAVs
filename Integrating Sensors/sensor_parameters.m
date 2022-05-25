% sensor parameters
SENSOR.sigma_gyro       = deg2rad(0.13);
SENSOR.sigma_accel      = 0.0025*MAV.gravity;
SENSOR.sigma_press_abs  = 10;
SENSOR.beta_press_abs   = 125;
SENSOR.sigma_press_diff = 2;
SENSOR.beta_press_diff  = 20;
SENSOR.Ts_compass       = 0.125;
SENSOR.sigma_magneto    = 0.3;
SENSOR.beta_magneto     = 1;

% GPS parameters
GPS.Ts      = 1;
GPS.sigma_v = 0.01;
GPS.sigma_n = 0.21;
GPS.sigma_e = 0.21;
GPS.sigma_d = 0.4;
GPS.k       = 1/1100;