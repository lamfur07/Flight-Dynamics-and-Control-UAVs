% estimate_states_simple
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%     GPS.
%   - performs similarly to standard EKF in the book, but GPS smoothing EKF
%     is simplified. Equations for north and east components are not
%     estimated, but rather calculated from estimates of Vg, psi, and chi,
%     and measurement of Va. EKF equations do not include wind states.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%            2/20/2018 - TM
%

function xhat = estimate_states_simple(uu, MAV, SENSOR, SIM)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
 
   
    % define persistent variables
    persistent alpha  % constant for low pass filter - only compute once
    persistent alpha1  % constant for low pass filter - only compute once
    persistent lpf_gyro_x   % low pass filter of x-gyro
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent lpf_gyro_z   % low pass filter of z-gyro
    persistent lpf_static   % low pass filter of static pressure sensor
    persistent lpf_diff     % low pass filter of diff pressure sensor
    persistent lpf_accel_x  % low pass filter of x-accelerometer
    persistent lpf_accel_y  % low pass filter of y-accelerometer
    persistent lpf_accel_z  % low pass filter of z-accelerometer
    persistent xhat_a       % estimate of roll and pitch
    persistent P_a          % error covariance for roll and pitch angles
    persistent xhat_p       % estimate of pn, pe, Vg, chi, psi
    persistent P_p          % error covariance for pn, pe, Vg, chi, psi
    persistent y_gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent y_gps_e_old  % last measurement of gps_e - used to detect new GPS signal
    persistent y_gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    persistent y_gps_course_old  % last measurement of gps_course - used to detect new GPS signal
    
   
    % initialize persistent variables
    lpf_a = 50;
    lpf_a1 = 20;
    if t==0
        alpha             = exp(-lpf_a*SIM.ts_control); 
        alpha1            = exp(-lpf_a1*SIM.ts_control);
        lpf_gyro_x        = 0;
        lpf_gyro_y        = 0;
        lpf_gyro_z        = 0;
        lpf_static        = 0;
        lpf_diff          = 0;
        lpf_accel_x       = 0;
        lpf_accel_y       = 0;
        lpf_accel_z       = 0;
        xhat_a            = [MAV.phi0, MAV.theta0]';
        P_a               = diag([(15*pi/180)^2,(15*pi/180)^2]);
        xhat_p            = [MAV.pn0; MAV.pe0; MAV.u0; MAV.psi0; MAV.psi0];
        P_p               = diag([10^2, 10^2, 1^2, (10*pi/180)^2, (5*pi/180)^2]); 
        y_gps_n_old       = -9999;
        y_gps_e_old       = -9999;
        y_gps_Vg_old      = -9999;
        y_gps_course_old  = -9999;
    end
    
    %------------------------------------------------------------------
    % low pass filter gyros to estimate angular rates
    lpf_gyro_x = alpha*lpf_gyro_x + (1-alpha)*y_gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*y_gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z + (1-alpha)*y_gyro_z;
    phat = lpf_gyro_x;
    qhat = lpf_gyro_y;
    rhat = lpf_gyro_z;
    
    %------------------------------------------------------------------
    % low pass filter static pressure sensor and invert to estimate
    % altitude
    lpf_static = alpha1*lpf_static + (1-alpha1)*y_static_pres; %or alpha
    hhat = lpf_static/(MAV.density*MAV.gravity);
    
    % low pass filter diff pressure sensor and invert to estimate Va
    lpf_diff = alpha1*lpf_diff + (1-alpha1)*y_diff_pres; %or alpha
    Vahat = sqrt(2*lpf_diff/MAV.density);
    
        
    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate roll and pitch angles
    Q_a = 10^(-9)*diag([1,1]);
    R_accel = SENSOR.sigma_accel^2;
   
    N = 10;
    % prediction step
    for i=1:N
        Tp = SIM.ts_control/N;
        cp = cos(xhat_a(1));  % cos(phi)
        sp = sin(xhat_a(1));  % sin(phi)
        tt = tan(xhat_a(2));  % tan(theta)
        ct = cos(xhat_a(2));  % cos(theta)
        f_a = [phat + qhat*sp*tt + rhat*cp*tt;
               qhat*cp - rhat*sp];
        A_a = [qhat*cp*tt - rhat*sp*tt, (qhat*sp-rhat*cp/ct^2);
               -qhat*sp - rhat*cp, 0]; 
        xhat_a = xhat_a + (Tp*f_a);
        P_a = P_a + (Tp*(A_a*P_a + P_a*A_a' + Q_a));
    end
     % measurement updates
    cp = cos(xhat_a(1));  % cos(phi)
    sp = sin(xhat_a(1));  % sin(phi)
    ct = cos(xhat_a(2));  % cos(theta)
    st = sin(xhat_a(2));  % sin(theta)
    
    threshold = 2;
    % x-axis accelerometer
    h_a  = qhat*Vahat*st + MAV.gravity*st;
    C_a  = [0, qhat*Vahat*ct + MAV.gravity*ct];
    L_a  = P_a*C_a'*inv(R_accel+C_a*P_a*C_a');
    P_a  = (eye(2) - L_a*C_a)*P_a;   
    if norm(y_accel_x - h_a)<threshold
        xhat_a = xhat_a + L_a*(y_accel_x - h_a);
    end
    % y-axis accelerometer
    h_a  = rhat*Vahat*ct - phat*Vahat*st - MAV.gravity*ct*sp;
    C_a  = [-MAV.gravity*cp*ct,  -rhat*Vahat*st-phat*Vahat*ct+MAV.gravity*sp*st];
    L_a  = P_a*C_a'*inv(R_accel+C_a*P_a*C_a');
    P_a  = (eye(2) - L_a*C_a)*P_a; 
    if norm(y_accel_x - h_a)<threshold
       xhat_a = xhat_a + L_a*(y_accel_y - h_a);
    end
    % z-axis accelerometer
    h_a  = -qhat*Vahat*ct - MAV.gravity*ct*cp;
    C_a  = [MAV.gravity*sp*ct, (qhat*Vahat + MAV.gravity*cp)*st];
    L_a  = P_a*C_a'*inv(R_accel+C_a*P_a*C_a');
    P_a  = (eye(2) - L_a*C_a)*P_a; 
    if norm(y_accel_x - h_a)<threshold
        xhat_a = xhat_a + L_a*(y_accel_z - h_a);
    end
     
    phihat   = xhat_a(1);
    thetahat = xhat_a(2);

   
    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate pn, pe, Vg, chi, psi
    Q_p = diag([...
                .1,...    % pn
                .1,...    % pe
                .1,...    % Vg
                .0001,... % chi
                .001...   %psi
               ]);
           
    R_p = diag([...
                SENSOR.gps_n_sigma^2,...              % y_gps_n
                SENSOR.gps_e_sigma^2,...              % y_gps_e
                SENSOR.gps_Vg_sigma^2,...             % y_gps_Vg
                (SENSOR.gps_Vg_sigma/xhat_p(3))^2,... % y_gps_course
               ]);
    
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    psihat   = xhat_p(5);
           
    cc = cos(chihat); %cos(chi)
    sc = sin(chihat); %sin(chi)
    cp = cos(psihat); %cos(psi)
    sp = sin(psihat); %sin(psi)
    
    pndot        = Vghat*cc;
    pedot        = Vghat*sc;
    chidot       = (MAV.gravity/Vghat)*tan(phihat)*cos(chihat-psihat);
    wnhat        = Vghat*cos(chihat) - Vahat*cos(psihat);
    wehat        = Vghat*sin(chihat) - Vahat*sin(psihat);
    wndot        = 0;
    wedot        = 0;
    psidot       = qhat*(sin(phihat)/cos(thetahat)) + rhat*(cos(phihat)/cos(thetahat));
    Vgdot        = (Vahat*psidot*(wehat*cp - wnhat*sp))/Vghat;
    partialVgdot = (-psidot*Vahat*(wnhat*cp+wehat*sp))/Vghat;
    
    N = 10;
    % prediction step
    for i=1:N
        Tp  = SIM.ts_control/N;
        f_p = [pndot;
               pedot;
               Vgdot;
               chidot;
               psidot];
           
        A_p = [0, 0, cc, -Vghat*sc, 0;...
               0, 0, sc, Vghat*cc, 0;...
               0, 0, -Vgdot/Vghat, 0, partialVgdot;...
               0, 0, (-MAV.gravity/Vghat^2)*tan(phihat), 0, 0;
               0, 0, 0, 0, 0];
        xhat_p = xhat_p + Tp*f_p;
        P_p = P_p + Tp*(A_p*P_p + P_p*A_p' + Q_p);
    end
    
    % measurement updates
    if  (y_gps_n~=y_gps_n_old)...
        ||(y_gps_e~=y_gps_e_old)...
        ||(y_gps_Vg~=y_gps_Vg_old)...
        ||(y_gps_course~=y_gps_course_old)
        % gps North position
        h_p = pnhat;
        C_p = [1, 0, 0, 0, 0];
        L_p = P_p*C_p'*inv(R_p(1,1) + C_p*P_p*C_p');
        P_p = (eye(5) - L_p*C_p)*P_p; 
        xhat_p = xhat_p + L_p*(y_gps_n - h_p);
        % gps East position
        h_p = pehat;
        C_p = [0, 1, 0, 0, 0];
        L_p = P_p*C_p'*inv(R_p(2,2) + C_p*P_p*C_p');
        P_p = (eye(5) - L_p*C_p)*P_p; 
        xhat_p = xhat_p + L_p*(y_gps_e - h_p);
        % gps ground speed
        h_p = Vghat;
        C_p = [0, 0, 1, 0, 0];
        L_p = P_p*C_p'*inv(R_p(3,3) + C_p*P_p*C_p');
        P_p = (eye(5) - L_p*C_p)*P_p; 
        xhat_p = xhat_p + L_p*(y_gps_Vg - h_p);
        % gps course
        % wrap course measurement
        while (y_gps_course - xhat_p(4))>pi, y_gps_course = y_gps_course - 2*pi; end
        while (y_gps_course - xhat_p(4))<-pi, y_gps_course = y_gps_course + 2*pi; end
        h_p = chihat;
        C_p = [0, 0, 0, 1, 0];
        L_p = P_p*C_p'*inv(R_p(4,4) + C_p*P_p*C_p');
        P_p = (eye(5) - L_p*C_p)*P_p; 
        xhat_p = xhat_p + L_p*(y_gps_course - h_p);

        % update stored GPS signals
        y_gps_n_old      = y_gps_n;
        y_gps_e_old      = y_gps_e;
        y_gps_Vg_old     = y_gps_Vg;
        y_gps_course_old = y_gps_course;
    end
    
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    psihat   = xhat_p(5);
  
    % not estimating these states 
    alphahat = thetahat;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    wnhat    = Vghat*cos(chihat) - Vahat*cos(psihat);
    wehat    = Vghat*sin(chihat) - Vahat*sin(psihat);
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end
