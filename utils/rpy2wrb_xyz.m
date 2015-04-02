function [ wRb ] = rpy2wrb_xyz( varargin )
%RPY2WRB_XYZ Converts varargin angles to a rotation matrix from body to world
% z ---yaw ---psi --- 3
% y ---pith ---theta --- 2
% x ---roll ----phi  ---- 1
if nargin == 1
    phi   = varargin{1}(1);
    theta = varargin{1}(2);
    psi   = varargin{1}(3);
elseif nargin == 3;
    phi   = varargin{1};
    theta = varargin{2};
    psi   = varargin{3};
end

Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1];
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi), cos(phi)];

wRb = Rz * Ry * Rx;

end