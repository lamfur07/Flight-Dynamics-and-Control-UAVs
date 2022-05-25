% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RWB 
%   5/14/2010 - RWB
%   2/18/2019 - RWB

function y = gps(uu, SENSOR)

    % relabel the inputs
    Va      = uu(1);
    alpha   = uu(2);
    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
    u       = uu(10);
    v       = uu(11);
    w       = uu(12);
    phi     = uu(13);
    theta   = uu(14);
    psi     = uu(15);
    p       = uu(16);
    q       = uu(17);
    r       = uu(18);
    t       = uu(19);
    
    persistent vN;
    persistent vE;
    persistent vD;
    
    if t == 0
        vN = 0;
        vE = 0;
        vD = 0;
    end
    
    eta_gps_n = SENSOR.gps_n_sigma*randn();
    eta_gps_e = SENSOR.gps_e_sigma*randn();
    eta_gps_h = SENSOR.gps_h_sigma*randn();
    
    vN = exp(-SENSOR.gps_k*SENSOR.ts_gps)*vN + eta_gps_n;
    vE = exp(-SENSOR.gps_k*SENSOR.ts_gps)*vE + eta_gps_e;
    vD = exp(-SENSOR.gps_k*SENSOR.ts_gps)*vD + eta_gps_h;
        
    y_gps_n      = pn + vN;
    y_gps_e      = pe + vE;
    y_gps_h      = -pd + vD;
    
    V_n        = Va*cos(psi) + wn;
    V_e        = Va*sin(psi) + we;
    V_g        = sqrt(V_n^2 + V_e^2);
    course_chi = atan2(V_e, V_n);
    
    sigma_Vg  = SENSOR.gps_Vg_sigma;
    sigma_chi = sigma_Vg/V_g;
    
    eta_v = sigma_Vg*randn();
    eta_chi = sigma_chi*rand();
    
    y_gps_Vg     = V_g + eta_v;
    y_gps_course = course_chi + eta_chi;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



