function out = forces_moments(x, delta, wind, MAV)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % rotation matrix from inertial frame to body frame
    R       = [cos(theta)*cos(psi)                                  cos(theta)*sin(psi)                                 -sin(theta);
               sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)       sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)      sin(phi)*cos(theta);
               cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)       cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)      cos(phi)*cos(theta)];
    
    W_NED = [w_ns w_es w_ds];
    G_UVW = [u_wg v_wg w_wg];
    
    equation = R*W_NED' + G_UVW';
    
    % compute wind data in Body Frame
    WX = equation(1);
    WY = equation(2);
    WZ = equation(3);
    
    w_body = [WX; WY; WZ];
    
    % compute wind data in Inertial Frame
%     w_inertial = transpose(R)*w_body;
    w_inertial = (W_NED') + (R')*(G_UVW');
    
    w_n = w_inertial(1);
    w_e = w_inertial(2);
    w_d = w_inertial(3);
    
    % airspeed vector
    ur = u - w_body(1);
    vr = v - w_body(2);
    wr = w - w_body(3);
    
    v_ba = [ur; vr; wr];
    
    % compute air data
    Va    = norm(v_ba);
    Alpha = atan2(wr,ur);
    Beta  = asin(vr/Va);
    
    % Initializing constants sigmoid and CL(alpha)and stability derivative
    num     = 1 + exp(-MAV.M*(Alpha-MAV.alpha0)) + exp(MAV.M*(Alpha+MAV.alpha0));
    den     = (1+exp(-MAV.M*(Alpha-MAV.alpha0)))*(1+exp(MAV.M*(Alpha+MAV.alpha0)));
    sigmoid = num/den;

%     num1       = pi*MAV.AR;
%     den1       = 1+sqrt(1+(MAV.AR/2)^2);
%     stab_deriv = num1/den1;
    
    C_L_alpha = (1-sigmoid)*(MAV.C_L_0+(MAV.C_L_alpha*Alpha)) + sigmoid*(2*sign(Alpha)*((sin(Alpha))^2)*cos(Alpha));
    
    % Initializing constant CD(alpha)
    num2 = (MAV.C_L_0+(MAV.C_L_alpha*Alpha))^2;
    den2 = pi*MAV.e*MAV.AR;
    ratio = num2/den2;
    
    C_D_alpha = MAV.C_D_p + ratio;
    
    % Iniitializing coefficients to solve equations
    C_X_Alpha       = -C_D_alpha*cos(Alpha) + C_L_alpha*sin(Alpha);
    C_Xq_Alpha      = -MAV.C_D_q*cos(Alpha) + MAV.C_L_q*sin(Alpha);
    C_Xdele_Alpha   = -MAV.C_D_delta_e*cos(Alpha) + MAV.C_L_delta_e*sin(Alpha);
    C_Z_Alpha       = -C_D_alpha*sin(Alpha) - C_L_alpha*cos(Alpha);
    C_Z_q_Alpha     = -MAV.C_D_q*sin(Alpha) - MAV.C_L_q*cos(Alpha);
    C_Zdele_Alpha   = -MAV.C_D_delta_e*sin(Alpha) - MAV.C_L_delta_e*cos(Alpha);    
    
    % constants
    D5 = (MAV.D_prop)^5;
    D4 = (MAV.D_prop)^4;
    D3 = (MAV.D_prop)^3;
    D2 = (MAV.D_prop)^2;
    
    % propulsion thrust and torque constants
    Vin = MAV.V_max*delta_t;
    A = (MAV.density*D5*MAV.C_Q0)/((2*pi)^2);
    B = ((MAV.density*D4*MAV.C_Q1*Va)/2*pi) + (MAV.KQ^2/MAV.R_motor);
    C = (MAV.density*D3*MAV.C_Q2*(Va^2)) - (MAV.KQ*Vin/MAV.R_motor) + (MAV.KQ*MAV.i0);
    root1 = B^2;
    root2 = 4*A*C;
    Den = 2*A;
    root = sqrt(root1-root2);
    omega_p = (-B + root)/Den;
%     omega_p = max(roots([A,B,C]));
    
    % constant gravity
    g(1) = -MAV.mass*MAV.gravity*sin(theta);
    g(2) = MAV.mass*MAV.gravity*cos(theta)*sin(phi);
    g(3) = MAV.mass*MAV.gravity*cos(theta)*cos(phi);
    
    % longitudinal force
    f_long(1) = (0.5*MAV.density*(Va^2)*MAV.S_wing)*(C_X_Alpha + C_Xq_Alpha*(MAV.c*q/2*Va) + C_Xdele_Alpha*delta_e);
    f_long(2) = (0.5*MAV.density*(Va^2)*MAV.S_wing)*(MAV.C_Y_0 + MAV.C_Y_beta*Beta + MAV.C_Y_p*(MAV.b*p/2*Va) + MAV.C_Y_r*(MAV.b*r/2*Va) + MAV.C_Y_delta_a*delta_a + MAV.C_Y_delta_r*delta_r);
    f_long(3) = (0.5*MAV.density*(Va^2)*MAV.S_wing)*(C_Z_Alpha + C_Z_q_Alpha*(MAV.c*q/2*Va) + C_Zdele_Alpha*delta_e);
    
    % thrust force Tp
    Tp = (MAV.density*D4*MAV.C_T0*(omega_p^2))/(4*(pi^2)) + (MAV.density*D3*MAV.C_T1*omega_p*Va)/(2*pi) + (MAV.density*D2*MAV.C_T2*(Va^2));
    
    % thrust force
%     f_thr(1) = (0.5*MAV.density*MAV.S_prop*MAV.C_prop)*((MAV.k_motor*delta_t)^2 - (Va)^2);
    f_thr(1) = Tp;
    f_thr(2) = 0;
    f_thr(3) = 0;
    
    % lateral torque
    t_lat(1) = (0.5*MAV.density*(Va^2)*MAV.S_wing)* (MAV.b*(MAV.C_l_0 + MAV.C_l_beta*Beta + MAV.C_l_p*(MAV.b*p/2*Va) + MAV.C_l_r*(MAV.b*r/2*Va) + MAV.C_l_delta_a*delta_a + MAV.C_l_delta_r*delta_r));
    t_lat(2) = (0.5*MAV.density*(Va^2)*MAV.S_wing)* (MAV.c*(MAV.C_m_0 + MAV.C_m_alpha*Alpha + MAV.C_m_q*(MAV.c*q/2*Va) + MAV.C_m_delta_e*delta_e));
    t_lat(3) = (0.5*MAV.density*(Va^2)*MAV.S_wing)* (MAV.b*(MAV.C_n_0 + MAV.C_n_beta*Beta + MAV.C_n_p*(MAV.b*p/2*Va) + MAV.C_n_r*(MAV.b*r/2*Va) + MAV.C_n_delta_a*delta_a + MAV.C_n_delta_r*delta_r)); 
    
    % propulsion torque
    
    Qp = (MAV.density*D5*MAV.C_Q0*(omega_p^2))/(4*(pi^2)) + (MAV.density*D4*MAV.C_Q1*omega_p*Va)/(2*pi) + (MAV.density*D3*MAV.C_Q2*(Va^2));

%         t_prop(1) = 0;
    t_prop(1) = Qp;
    t_prop(2) = 0;
    t_prop(3) = 0;
    
    % Force matrix
    Force(1) =  g(1) + f_long(1) + f_thr(1);
    Force(2) =  g(2) + f_long(2) + f_thr(2);
    Force(3) =  g(3) + f_long(3) + f_thr(3);
    
    % Torque matrix
    Torque(1) =  t_lat(1) + t_prop(1);
    Torque(2) =  t_lat(2) + t_prop(2);
    Torque(3) =  t_lat(3) + t_prop(3);

    out = [Force'; Torque'; Va; Alpha; Beta; w_n; w_e; w_d];

end