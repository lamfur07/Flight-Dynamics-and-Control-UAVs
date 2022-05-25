function drawSpacecraftBody(uu)
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
    aircraftOrigin = drawBody(pn, pe, pd, phi, theta, psi, []);
    title('Box')
    xlabel('East')
    ylabel('North')
    zlabel('-Down')
    view(32,47)  % set the view angle for figure
    axis([-100*scale,100*scale,-100*scale,100*scale,-100*scale, 100*scale]);
    grid on
    
    % at every other time step, redraw box
else
    drawBody(pn, pe, pd, phi, theta, psi, aircraftOrigin);
end

end

function handle = drawBody(pn, pe, pd, phi, theta, psi, handle)
% define points on spacecraft in local NED coordinates
NED = spacecraftPoints;
% rotate spacecraft by phi, theta, psi
NED = rotate(NED, phi, theta,psi);
% transle spacecraft to [pn, pe, pd];
NED = translate(NED, pn, pe, pd);
% transform vertices form NED to XYZ
R = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1];
XYZ = R*NED;
% plot spacecraft
if isempty(handle)
    handle = plot3(XYZ(1,:), XYZ(2,:), XYZ(3,:));
else
    set(handle, 'XData', XYZ(1,:),'YData', XYZ(2,:),'ZData', XYZ(3,:));
    drawnow
end
end

function XYZ = rotate(XYZ, phi, theta, psi)
% define rotaion matrix
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
% rotate vertice
XYZ = R*XYZ;
end

function XYZ = translate(XYZ, pn, pe, pd)
XYZ = XYZ + repmat([pn;pe;pd], 1, size(XYZ,2));
end