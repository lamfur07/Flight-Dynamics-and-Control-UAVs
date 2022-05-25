% Defining a function to plot the points for the aircraft
function XYZ = spacecraftPoints

scale = 20;

% In local NED the points for the spacecraft is as follows
XYZ = [
    7    0     0;   % point 1
    4    1    -1;   % point 2
    4   -1    -1;   % point 3
    7    0     0;   % point 1
    4   -1    -1;   % point 3
    4   -1     1;   % point 4
    7    0     0;   % point 1
    4   -1     1;   % point 4
    4    1     1;   % point 5
    7    0     0;   % point 1
    4    1     1;   % point 5
    4    1    -1;   % point 2
  -15    0     0;   % point 6
    4   -1    -1;   % point 3
  -15    0     0;   % point 6
    4   -1     1;   % point 4
    4    1     1;   % point 5
  -15    0     0;   % point 6
  -15    0    -3;   % point 16
  -12    0     0;   % point 15
  -15    0     0;   % point 6
   -6    0     0;   % interim point
   -6   10     0;   % point 8
    0   10     0;   % point 7
    0  -10     0;   % point 10
   -6  -10     0;   % point 9
   -6    0     0;   % interim point
  -15    0     0;   % point 6
  -15    5     0;   % point 12
  -12    5     0;   % point 11
  -12   -5     0;   % point 14
  -15   -5     0;   % point 13
  -15    5     0;   % point 12
  -15   -5     0;   % point 13
  -15    0     0;   % point 6
     ]';
 
XYZ = scale*XYZ;

end