function y = autopilot(uu, AP)
%
% autopilot for mavsim
%
% Modification History:
%   2/16/2019 - RWB
%   10/24/2021 - RR

% process inputs
NN       = 0;
pn       = uu(1+NN);  % inertial North position
pe       = uu(2+NN);  % inertial East position
h        = uu(3+NN);  % altitude
Va       = uu(4+NN);  % airspeed
alpha    = uu(5+NN);  % angle of attack
beta     = uu(6+NN);  % side slip angle
phi      = uu(7+NN);  % roll angle
theta    = uu(8+NN);  % pitch angle
chi      = uu(9+NN);  % course angle
p        = uu(10+NN); % body frame roll rate
q        = uu(11+NN); % body frame pitch rate
r        = uu(12+NN); % body frame yaw rate
Vg       = uu(13+NN); % ground speed
wn       = uu(14+NN); % wind North
we       = uu(15+NN); % wind East
psi      = uu(16+NN); % heading
bx       = uu(17+NN); % x-gyro bias
by       = uu(18+NN); % y-gyro bias
bz       = uu(19+NN); % z-gyro bias
NN       = NN+19;
Va_c     = uu(1+NN);  % commanded airspeed (m/s)
h_c      = uu(2+NN);  % commanded altitude (m)
chi_c    = uu(3+NN);  % commanded course (rad)
phi_c_ff = uu(4+NN);  % feedforward roll command (rad)

NN       = NN+4;
t        = uu(1+NN);   % time

%----------------------------------------------------------
% lateral autopilot
chi_ref = wrap(chi_c, chi);
if t==0
    phi_c = phi_c_ff + course_with_roll(chi_ref, chi, 1, AP);
    delta_r = yaw_damper(r, 1, AP);
else
    phi_c   = phi_c_ff + course_with_roll(chi_ref, chi, 0, AP);
    delta_r = yaw_damper(r, 0, AP);
end
phi_c   = sat(phi_c, 45*pi/180, -45*pi/180);
delta_a = roll_with_aileron(phi_c, phi, p, AP);

%----------------------------------------------------------
% longitudinal autopilot

h_ref = sat(h_c, h+AP.altitude_zone, h-AP.altitude_zone);
if t==0
%     delta_t = 0;
%     theta_c = 0;
    delta_t = airspeed_with_throttle(Va_c, Va, 1, AP);
    theta_c = altitude_with_pitch(h_ref, h, 1, AP);
else
    delta_t = airspeed_with_throttle(Va_c, Va, 0, AP);
    theta_c = altitude_with_pitch(h_ref, h, 0, AP);
end
delta_e = pitch_with_elevator(theta_c, theta, q, AP);

% limit range of throttle setting to [0,1]
delta_t = sat(delta_t, 1, 0);

%----------------------------------------------------------
% create outputs
% control outputs
delta = [delta_e; delta_a; delta_r; delta_t];
% commanded (desired) states
x_command = [...
    0;...                    % pn
    0;...                    % pe
    h_c;...                  % h
    Va_c;...                 % Va
    0;...                    % alpha
    0;...                    % beta
    phi_c;...                % phi
    theta_c;...              % theta
    chi_c;...                % chi
    0;...                    % p
    0;...                    % q
    0;...                    % r
    ];

y = [delta; x_command];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_with_roll
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c_sat = course_with_roll(chi_c, chi, flag, AP)
persistent integrator_phi;
persistent error_d1_phi;

if flag==1
    integrator_phi = 0;
    error_d1_phi = 0;
    
end

error = chi_c - chi;
integrator_phi = integrator_phi + (AP.Ts/2)*(error + error_d1_phi);

error_d1_phi = error;

phi_c = AP.course_kp*error + AP.course_ki*integrator_phi;
u = sat(phi_c, 45*pi/180, -45*pi/180);

phi_c_sat = u;

% implement integrator anti-windup
if AP.course_ki~=0
    k_antiwindup = AP.Ts/AP.course_ki;
    integrator_phi = integrator_phi + k_antiwindup*(phi_c_sat-phi_c);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_with_aileron
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_with_aileron(phi_c, phi, p, AP)
error = phi_c - phi;
u = sat(AP.roll_kp*error - AP.roll_kd*p, 45*pi/180, -45*pi/180);
delta_a = u;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_with_elevator
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_with_elevator(theta_c, theta, q, AP)
error = theta_c - theta;
del_e = AP.pitch_kp*error - AP.pitch_kd*q;
u = sat(del_e, 45*pi/180, -45*pi/180);
delta_e = u;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function delta_t_sat = airspeed_with_throttle(Va_c, Va, flag, AP)
persistent integrator_delta_t;
persistent error_d1_delta_t;

if flag==1
    integrator_delta_t = 0;
    error_d1_delta_t = 0;
end

error = Va_c - Va;
integrator_delta_t = integrator_delta_t + (AP.Ts/2)*(error + error_d1_delta_t);
error_d1_delta_t = error;

delta_t = AP.airspeed_throttle_kp*error + AP.airspeed_throttle_ki*integrator_delta_t;
u = sat(delta_t, 1, 0);

delta_t_sat = u;

% implement integrator anti-windup
if AP.airspeed_throttle_ki~=0
    k_antiwindup = AP.Ts/AP.airspeed_throttle_ki;
    integrator_delta_t = integrator_delta_t + k_antiwindup*(delta_t_sat-delta_t);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c_sat = altitude_with_pitch(h_c, h, flag, AP)
persistent integrator_theta_c;
persistent error_d1_theta_c;

if flag==1
    integrator_theta_c = 0;
    error_d1_theta_c = 0;
end

error = h_c - h;

integrator_theta_c = integrator_theta_c + (AP.Ts/2)*(error + error_d1_theta_c);

error_d1_theta_c = error;

theta_c = AP.altitude_kp*error + AP.altitude_ki*integrator_theta_c;
% u = sat(theta_c, 30*pi/180, -30*pi/180);
u = sat(theta_c, AP.up_limit, AP.low_limit);

theta_c_sat = u;

% implement integrator anti-windup
if AP.altitude_ki~=0
    k_antiwindup = AP.Ts/AP.altitude_ki;
    integrator_theta_c = integrator_theta_c + k_antiwindup*(theta_c_sat-theta_c);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% yaw_damper
%   - yaw rate with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_r = yaw_damper(r, flag, AP)
persistent integrator;
% initialize persistent variables at beginning of simulation
if flag==1
    integrator = 0;
end
% update integrator
integrator = integrator...
    + AP.Ts*(-integrator / AP.yaw_damper_tau_r + AP.yaw_damper_kp * r);
% yaw damper
delta_r = -integrator / AP.yaw_damper_tau_r + AP.yaw_damper_kp*r;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
if in > up_limit
    out = up_limit;
elseif in < low_limit
    out = low_limit;
else
    out = in;
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = wrap(out, in)
while out-in > pi
    out = out - 2*pi;
end
while out-in < -pi
    out = out + 2*pi;
end
end

