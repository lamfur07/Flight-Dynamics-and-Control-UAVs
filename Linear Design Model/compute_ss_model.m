% x_trim is the trimmed state,
% u_trim is the trimmed input
  
  
[A,B,C,D]=linmod('mavsim_trim',P.x_trim,P.u_trim);

% matrix for lateral state space model
E1 = [...
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0];

E2 = [...
    0, 1, 0, 0;
    0, 0, 1, 0];

% extracting lateral coefficients
A_lat = E1*A*E1';
B_lat = E1*B*E2';

% matrix for longitudinal state space model
F1 = [...
    0, 0,  0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    0, 0,  1, 0, 0, 0, 0, 0, 0, 0, 0, 0];

F2 = [...
    1, 0, 0, 0;
    0, 0, 0, 1];

% extracting longitudinal coefficients
A_lon = F1*A*F1';
B_lon = F1*B*F2';

% eigenvalues for A longitudinal and A lateral
eigen_A_long = eig(A_lon);
eigen_A_late = eig(A_lat);

omega_n1 = abs(eigen_A_long(2));
zeta1 = real(eigen_A_long(2))/omega_n1;

% omega_n2 = abs(eigen_A_late(4));
% zeta2 = real(Eigen_A_late(4))/omega_n2;


  