% initialize the mav viewer
% addpath('../tools');  

% initial conditions
MAV.pn0    = 0;     % initial North position
MAV.pe0    = 0;     % initial East position
MAV.pd0    = -100;  % initial Down position (negative altitude)
MAV.u0     = 25;    % initial velocity along body x-axis
MAV.v0     = 0;     % initial velocity along body y-axis
MAV.w0     = 0;     % initial velocity along body z-axis
MAV.phi0   = 0;     % initial roll angle
MAV.theta0 = 0;     % initial pitch angle
MAV.psi0   = 0;     % initial yaw angle
e          = Euler2Quaternion(MAV.phi0, MAV.theta0, MAV.psi0);
MAV.e0     = e(1);  % initial quaternion
MAV.e1     = e(2);
MAV.e2     = e(3);
MAV.e3     = e(4);
MAV.p0     = 0;     % initial body frame roll rate
MAV.q0     = 0;     % initial body frame pitch rate
MAV.r0     = 0;     % initial body frame yaw rate
   
%physical parameters of airframe
MAV.gravity       = 9.81;
MAV.mass          = 11;
MAV.Jx            = 0.824;
MAV.Jy            = 1.135;
MAV.Jz            = 1.759;
MAV.Jxz           = 0.120;
MAV.S_wing        = 0.55;
MAV.b             = 2.9;
MAV.c             = 0.19;
MAV.S_prop        = 0.2027;
MAV.C_prop        = 1.0;
MAV.density       = 1.2682;
MAV.rho           = 1.2682;
MAV.k_motor       = 80;
MAV.k_tp          = 0;
MAV.k_omega       = 0;
MAV.e             = 0.9;
MAV.AR            = MAV.b^2/MAV.S_wing;

% Gamma parameters from uavbook page 36
MAV.Gamma  = MAV.Jx*MAV.Jz-MAV.Jxz^2;
MAV.Gamma1 = (MAV.Jxz*(MAV.Jx-MAV.Jy+MAV.Jz))/MAV.Gamma;
MAV.Gamma2 = (MAV.Jz*(MAV.Jz-MAV.Jy)+MAV.Jxz*MAV.Jxz)/MAV.Gamma;
MAV.Gamma3 = MAV.Jz/MAV.Gamma;
MAV.Gamma4 = MAV.Jxz/MAV.Gamma;
MAV.Gamma5 = (MAV.Jz-MAV.Jx)/MAV.Jy;
MAV.Gamma6 = MAV.Jxz/MAV.Jy;
MAV.Gamma7 = (MAV.Jx*(MAV.Jx-MAV.Jy)+MAV.Jxz*MAV.Jxz)/MAV.Gamma;
MAV.Gamma8 = MAV.Jx/MAV.Gamma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients
MAV.C_L_0         = 0.23;
MAV.C_D_0         = 0.043;
MAV.C_m_0         = 0.0135;
MAV.C_L_alpha     = 5.61;
MAV.C_D_alpha     = 0.030;
MAV.C_m_alpha     = -2.74;
MAV.C_L_q         = 7.95;
MAV.C_D_q         = 0.0;
MAV.C_m_q         = -38.21;
MAV.C_L_delta_e   = 0.13;
MAV.C_D_delta_e   = 0.0135;
MAV.C_m_delta_e   = -0.99;
MAV.M             = 50;
MAV.alpha0        = 0.47;
MAV.epsilon       = 0.16;
MAV.C_D_p         = 0.0;

MAV.C_Y_0         = 0.0;
MAV.C_l_0         = 0.0;
MAV.C_n_0         = 0.0;
MAV.C_Y_beta      = -0.83;
MAV.C_l_beta      = -0.13;
MAV.C_n_beta      = 0.073;
MAV.C_Y_p         = 0.0;
MAV.C_l_p         = -0.51;
MAV.C_n_p         = -0.069;
MAV.C_Y_r         = 0.0;
MAV.C_l_r         = 0.25;
MAV.C_n_r         = -0.095;
MAV.C_Y_delta_a   = 0.075;
MAV.C_l_delta_a   = 0.17;
MAV.C_n_delta_a   = -0.011;
MAV.C_Y_delta_r   = 0.19;
MAV.C_l_delta_r   = 0.0024;
MAV.C_n_delta_r   = -0.069;

% Parameters for propulsion thrust and torque models
MAV.D_prop = 0.5;     % prop diameter in m

% Motor parameters
MAV.K_V         = 145;                     % from datasheet RPM/V
MAV.KQ          = (1/MAV.K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
MAV.R_motor     = 0.042;                   % ohms
MAV.i0          = 1.5;                     % no-load (zero-torque) current (A)

% Inputs
MAV.ncells = 12;
MAV.V_max  = 3.7*MAV.ncells;       % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
MAV.C_Q2 = -0.01664;
MAV.C_Q1 = 0.004970;
MAV.C_Q0 = 0.005230;

MAV.C_T2 = -0.1079;
MAV.C_T1 = -0.06044;
MAV.C_T0 = 0.09357;

function quaternion = Euler2Quaternion(phi, theta, psi)
    % converts euler angles to a quaternion
    e0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
    e1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
    e2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
    e3 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);
    quaternion = [e0; e1; e2; e3];
end