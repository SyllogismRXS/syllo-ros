function [xdot] = videoray_model(t,x)
%==============================================================================
% Title  : VideoRay Dynamics Model
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
%==============================================================================
% State Definition:
% 1:  u     : surge velocity
% 2:  v     : sway velocity
% 3:  w     : heave velocity
% 4:  p     : roll rate
% 5:  q     : pitch rate
% 6:  r     : yaw rate
% 7:  xpos  : earth x-pos
% 8:  ypos  : earth y-pos
% 9:  zpos  : earth z-pos
% 10: phi   : roll angle
% 11: theta : pitch angle
% 12: psi   : yaw angle
% 13: Thrust_X : Forward thrust
% 14: Thrust_Theta : Yaw Thrust
% 15: Thrust_Z : Vertical Thrust
%==============================================================================
u     = x(1);
v     = x(2);
w     = x(3);
p     = x(4);
q     = x(5);
r     = x(6);
xpos  = x(7);
ypos  = x(8);
zpos  = x(9);
phi   = x(10);
theta = x(11);
psi   = x(12);

%% VideoRay Specific Model

% Added Mass Terms
global X_udot;
global Y_vdot;
global Z_wdot;
global N_rdot;              

% Linear Drag Coefficients
global Xu;
global Yv;
global Nr;
global Zw;

% Quadratic Drag Coefficients
global Xuu;
global Yvv;
global Nrr;
global Zww;

% Select the appropriate controller based on user input
global controller_id;
global DEMO = 1;
global PROPORTIONAL = 2;

if controller_id == DEMO
    [X N Z] = control_openloop(t,x);
elseif controller_id == PROPORTIONAL
    [X N Z] = control_p(t,x);
else
    error("Invalid controller selected")
end

% Log the thruster inputs for later plotting
xdot(13) = X;
xdot(14) = N;
xdot(15) = Z;

% Calculate fixed frame velocity rates
xdot(1) = (-Y_vdot*v*r + Xu*u + Xuu*u*abs(u) + X) / X_udot;
xdot(2) = (X_udot*u*r + Yv*v + Yvv*v*abs(v)) / Y_vdot;
xdot(3) = (Zw*w + Zww*w*abs(w) + Z) / Z_wdot;

% Calculate fixed frame orientation rates
xdot(4) = 0; % Assumes zero roll
xdot(5) = 0; % Assumes zero pitch
xdot(6) = (Nr*r + Nrr*r*abs(r) + N) / N_rdot;

%% END VIDEORAY SPECIFIC

% Precalculate trig functions
c1 = cos(phi);
c2 = cos(theta); 
c3 = cos(psi); 
s1 = sin(phi); 
s2 = sin(theta); 
s3 = sin(psi); 
t2 = tan(theta);

% Calculate inertial frame position
xdot(1,7) = c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w;
xdot(1,8) = s3*c2*u + (c1*c3+s1*s2*s3)*v + (c1*s2*s3-c3*s1)*w;
xdot(1,9) = -s2*u + c2*s1*v + c1*c2*w;

% Calculate inertial frame orientations
xdot(1,10) = p + (q*s1 + r*c1)*t2;
xdot(1,11) = q*c1 - r*s1;
xdot(1,12) = (q*s1 + r*c1)*sec(theta);

endfunction;
