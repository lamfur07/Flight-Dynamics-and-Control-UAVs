function drawSpacecraftBodyVFC(uu)
scale = 20;
% process inputs to function
pn       = uu(1)*scale;       % inertial North position
pe       = uu(2)*scale;       % inertial East position
pd       = uu(3)*scale;       % inertial Down position
phi      = uu(4);             % roll angle
theta    = uu(5);             % pitch angle
psi      = uu(6);             % yaw angle
t        = uu(7);             % time

% define persistent variables
persistent aircraftOrigin;

if t==0
    figure(1); clf;
    aircraftOrigin = drawSpacecraftBody(pn, pe, pd, phi, theta, psi, []);
    title('Box')
    xlabel('East')
    ylabel('North')
    zlabel('-Down')
    view(32,47)  % set the view angle for figure
    axis([-100*scale,100*scale,-100*scale,100*scale,-100*scale, 100*scale]);
    grid on
    
    % at every other time step, redraw box
else
    drawSpacecraftBody(pn, pe, pd, phi, theta, psi, aircraftOrigin);
end

end
%
function handle = drawSpacecraftBody(pn,pe,pd,phi,theta,psi,handle)
[V, F, patchcolors] = spacecraftVFC;
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
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi)];
R_pitch = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta)];
R_yaw = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1];
R = R_roll*R_pitch*R_yaw;
% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose
R = R';

% rotate vertices
pts = R*pts;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end
