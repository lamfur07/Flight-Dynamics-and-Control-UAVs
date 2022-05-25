% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RWB 
%   5/14/2010 - RWB
%   2/18/2019 - RWB
function y = sensors(uu, MAV, SENSOR)

    % relabel the inputs
    pn      = uu(1);
    pe      = uu(2);
    pd      = uu(3);
    u       = uu(4);
    v       = uu(5);
    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
    M_l     = uu(16);
    M_m     = uu(17);
    M_n     = uu(18);
    Va      = uu(19);
    alpha   = uu(20);
    beta    = uu(21);
    wn      = uu(22);
    we      = uu(23);
    wd      = uu(24);
    
    eta_gyro_x = SENSOR.sigma_gyro*randn();
    eta_gyro_y = SENSOR.sigma_gyro*randn();
    eta_gyro_z = SENSOR.sigma_gyro*randn();
    y_gyro_x   = p + eta_gyro_x;
    y_gyro_y   = q + eta_gyro_y;
    y_gyro_z   = r + eta_gyro_z;
    
    eta_accel_x = SENSOR.sigma_accel*randn();
    eta_accel_y = SENSOR.sigma_accel*randn();
    eta_accel_z = SENSOR.sigma_accel*randn();
    y_accel_x   = (F_x/MAV.mass) + (MAV.gravity*sin(theta)) + eta_accel_x;
    y_accel_y   = (F_y/MAV.mass) - (MAV.gravity*cos(theta)*sin(phi)) + eta_accel_y;
    y_accel_z   = (F_z/MAV.mass) - (MAV.gravity*cos(theta)*cos(phi)) + eta_accel_z;
    
    eta_press_abs  = SENSOR.sigma_press_abs*randn(); 
    y_static_pres = (MAV.density*MAV.gravity*pd) + SENSOR.beta_press_abs + eta_press_abs;
    
    eta_press_diff = SENSOR.sigma_press_diff*randn();
    y_diff_pres = (MAV.density*Va^2)/2 + SENSOR.beta_press_diff + eta_press_diff;

    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



