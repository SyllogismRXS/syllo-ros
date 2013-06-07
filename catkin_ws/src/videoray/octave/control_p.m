function [X N Z] = control_p(t,x)
%==============================================================================
% Title  : Simple proportional controller
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
% Notes  : 
%==============================================================================
global depth_ref;
global heading_ref;
global speed_ref;

X = 0;
N = 0;
Z = 0;

depth_err = depth_ref - x(9);
heading_err = heading_ref*pi/180 - x(12);
speed_err = speed_ref - x(1);

Kp = 5;

X = Kp*speed_err;
N = Kp*heading_err;
Z = Kp*depth_err;

endfunction
