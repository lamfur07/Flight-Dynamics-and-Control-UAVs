% x_trim is the trimmed state,
% u_trim is the trimmed input

% defining constants (shorter names)
D5 = MAV.D_prop^5;
D4 = MAV.D_prop^4;
D3 = MAV.D_prop^3;
D2 = MAV.D_prop^2;

% defining trim conditions
Va_trim      = P.y_trim(1);
theta_trim   = P.x_trim(8);
delta_t_trim = P.u_trim(4);
delta_e_trim = P.u_trim(1);
alpha_trim   = P.y_trim(2);

% initializing non-linear coefficients for transfer function calculation
CP_o       = MAV.Gamma3*MAV.C_l_0 + MAV.Gamma4*MAV.C_n_0;
CP_beta    = MAV.Gamma3*MAV.C_l_beta + MAV.Gamma4*MAV.C_n_beta;
CP_p       = MAV.Gamma3*MAV.C_l_p + MAV.Gamma4*MAV.C_n_p;
CP_r       = MAV.Gamma3*MAV.C_l_r + MAV.Gamma4*MAV.C_n_r;
CP_delta_a = MAV.Gamma3*MAV.C_l_delta_a + MAV.Gamma4*MAV.C_n_delta_a;
CP_delta_r = MAV.Gamma3*MAV.C_l_delta_r + MAV.Gamma4*MAV.C_n_delta_r;
CR_o       = MAV.Gamma4*MAV.C_l_0 + MAV.Gamma8*MAV.C_n_0;
CR_beta    = MAV.Gamma4*MAV.C_l_beta + MAV.Gamma8*MAV.C_n_beta;
CR_p       = MAV.Gamma4*MAV.C_l_p + MAV.Gamma8*MAV.C_n_p;
CR_r       = MAV.Gamma4*MAV.C_l_r + MAV.Gamma8*MAV.C_n_r;
CR_delta_a = MAV.Gamma4*MAV.C_l_delta_a + MAV.Gamma8*MAV.C_n_delta_a;
CR_delta_r = MAV.Gamma4*MAV.C_l_delta_r + MAV.Gamma8*MAV.C_n_delta_r;

% lateral transfer function - Roll
a_phi1 = -0.5*MAV.density*(Va_trim^2)*MAV.S_wing*MAV.b*CP_p*MAV.b/(2*Va_trim);
a_phi2 = 0.5*MAV.density*(Va_trim^2)*MAV.S_wing*MAV.b*CP_delta_a;

% lateral transfer function - Sideslip
a_beta1 = -MAV.density*Va_trim*MAV.S_wing*MAV.C_Y_beta/(2*MAV.mass);
a_beta2 = MAV.density*Va_trim*MAV.S_wing*MAV.C_Y_delta_r/(2*MAV.mass);

% longitudinal transfer function - Pitch Angle
a_theta1 = -MAV.density*(Va_trim^2)*MAV.c*MAV.S_wing*MAV.C_m_q*MAV.c/(2*MAV.Jy*2*Va_trim);
a_theta2 = -MAV.density*(Va_trim^2)*MAV.c*MAV.S_wing*MAV.C_m_0/(2*MAV.Jy);
a_theta3 = MAV.density*(Va_trim^2)*MAV.c*MAV.S_wing*MAV.C_m_delta_e/(2*MAV.Jy);

% Partial derivative of Tp
% First define omega_p (by terms and substituting Vmax*delta_t for Vin)
a  = MAV.density*D5*MAV.C_Q0/((2*pi)^2);
b1 = MAV.density*D4*MAV.C_Q1/(2*pi);
b2 = (MAV.KQ^2)/MAV.R_motor;
c1 = MAV.density*D3*MAV.C_Q2;
c2 = MAV.KQ*MAV.V_max/MAV.R_motor;
c3 = MAV.KQ*MAV.i0;

% Second, define Tp equation (term by term)
term_1 = MAV.density*D4*MAV.C_T0/(4*(pi^2));
term_2 = MAV.density*D3*MAV.C_T1/(2*pi);
term_3 = MAV.density*D2*MAV.C_T2;

% Third, create symbolic expression for Va and delta_t in Tp equation
% using the previously defined omega_p
syms Va delta_t
Tp_equation = term_1*((-(b1*Va + b2)+sqrt((b1*Va +b2)^2 - 4*a*(c1*Va^2 - c2*delta_t + c3)))/2*a)^2 + term_2*Va*((-(b1*Va + b2)+sqrt((b1*Va +b2)^2 - 4*a*(c1*Va^2 - c2*delta_t + c3)))/2*a) + term_3*Va^2;

% Fourth, find partial derivative
partial_Va      = diff(Tp_equation,Va);
partial_delta_t = diff(Tp_equation,delta_t);

% Fifth, substitute Va_trim and delta_t_trim in the partial derivative
Tp_partial_Va      = double(subs(partial_Va,[Va,delta_t],[Va_trim,delta_t_trim]));
Tp_partial_delta_t = double(subs(partial_delta_t,[Va,delta_t],[Va_trim,delta_t_trim]));

% longitudinal transfer function - Airspeed
a_V1 = (MAV.density*Va_trim*MAV.S_wing/MAV.mass)*(MAV.C_D_0+MAV.C_D_alpha*alpha_trim+MAV.C_D_delta_e*delta_e_trim) - Tp_partial_Va/MAV.mass;
a_V2 = (1/MAV.mass)*Tp_partial_delta_t;
a_V3 = MAV.gravity*cos(theta_trim-alpha_trim);

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([MAV.gravity/Va_trim],[1,0]);
T_v_delta_r     = tf([a_beta2],[1,a_beta1]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
