function [V,F,patchcolors] = spacecraftVFC
scale = 20;
% Define the vertices (physical location of vertices
V = [
     7    0   0; % point 1
     4    1  -1; % point 2
     4   -1  -1; % point 3
     4   -1   1; % point 4
     4    1   1; % point 5
   -15    0   0; % point 6
     0   10   0; % point 7
    -6   10   0; % point 8
    -6  -10   0; % point 9
     0  -10   0; % point 10
   -12    5   0; % point 11
   -15    5   0; % point 12
   -15   -5   0; % point 13
   -12   -5   0; % point 14
   -12    0   0; % point 15
   -15    0  -3; % point 16
    ];

V = V*scale;

% define faces as a list of vertices numbered above
F = [
    1,   2,   3,   1; % front
    1,   2,   5,   1; % front  
    1,   3,   4,   1; % front
    2,   3,   6,   2; % top
    7,   8,   9,  10; % top
   11,  12,  13,  14; % top
    2,   5,   6,   2; % right
    6,  15,  16,   6; % right
    3,   4,   6,   3; % left
    5,   4,   6,   5; % bottom
    ];

% define colors for each face
myred       = [1, 0, 0];
mygreen     = [0, 1, 0];
myblue      = [0, 0, 1];
myyellow    = [1, 1, 0];
mymagenta   = [0, 1, 1];

patchcolors = [
    myred;     % front
    myred;     % front
    myred;     % front
    mygreen;   % top
    mygreen;   % top
    mygreen;   % top
    myblue;    % right
    myblue;    % right
    myyellow;  % left
    mymagenta; % bottom
    ];
end