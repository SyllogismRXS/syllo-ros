function [X N Z] = control_openloop(t,x)
%==============================================================================
% Title  : Simple open loop controller
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
% Notes  : 
%==============================================================================

% Very simple control law that enables thrusters for 0.5 seconds, then
% disables all thrusters. This is just to test the system dynamics.
% X : input to the forward (fixed-body X) acceleration
% N : input to the heading (theta) acceleration
% Z : input to the vertical (Z) acceleration
if t < 0.5
    X = 1; % forward thrust
    N = 1; % turning thrust
    Z = -1; % vertical thrust
else
    X = 0; % forward thrust
    N = 0; % turning thrust
    Z = 0; % vertical thrust
end

endfunction
