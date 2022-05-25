function drawSpacecraftBodyVFC(uu)
scale = 20;
% process inputs to function
pn       = uu(1);       % inertial North position
pe       = uu(2);       % inertial East position
pd       = uu(3);       % inertial Down position
phi      = uu(4);       % roll angle
theta    = uu(5);       % pitch angle
psi      = uu(6);       % yaw angle
t        = uu(7);       % time

% define persistent variables
    persistent aircraftOrigin;
    persistent Vertices;
    persistent Faces;
    persistent facecolors;

if t==0
    figure(1); clf;
    [Vertices, Faces, facecolors] = defineSpaceCraftBody;
    aircraftOrigin = drawSpacecraftBody(Vertices, Faces, facecolors, pn, pe, pd, phi, theta, psi, []);
    title('SpaceCraft')
    xlabel('East')
    ylabel('North')
    zlabel('-Down')
    view(32,47)  % set the view angle for figure
    axis([-50*scale,50*scale,-50*scale,50*scale,-50*scale, 50*scale]);
    grid on
    
    % at every other time step, redraw box
else
    drawSpacecraftBody(Vertices, Faces, facecolors, pn, pe, pd, phi, theta, psi, aircraftOrigin);
end

end
%
function handle = drawSpacecraftBody(V,F,patchcolors,pn,pe,pd,phi,theta,psi,handle)

V = rotate(V', phi, theta, psi)';  % rotate vehicle
V = translate(V', pn, pe, pd)';  % translate vehicle
% transform vertices from NED to XYZ (for matlab rendering)
R = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1;...
    ];
V = V*R;

if isempty(handle)
    handle = patch('Vertices', V, 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat');
else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
end
end
%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

% define rotation matrix (right handed)
R_roll = [...
    1, 0, 0;...
    0, cos(phi), -sin(phi);...
    0, sin(phi), cos(phi)];
R_pitch = [...
    cos(theta), 0, sin(theta);...
    0, 1, 0;...
    -sin(theta), 0, cos(theta)];
R_yaw = [...
    cos(psi), -sin(psi), 0;...
    sin(psi), cos(psi), 0;...
    0, 0, 1];
R = R_roll*R_pitch*R_yaw;
% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose
% R = R';

% rotate vertices
pts = R*pts;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end

function [V,F,patchcolors] = defineSpaceCraftBody()
 scale = 20;
% Define the vertices (physical location of vertices)
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
