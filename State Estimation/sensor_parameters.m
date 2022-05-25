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

SENSOR.ts_gps = 1.0;
SENSOR.gps_k = 1 / 1100;  % 1 / s
SENSOR.gps_n_sigma = 0.21;
SENSOR.gps_e_sigma = 0.21;
SENSOR.gps_h_sigma = 0.40;
SENSOR.gps_Vg_sigma = 0.01;
SENSOR.gps_course_sigma = SENSOR.gps_Vg_sigma / 10;