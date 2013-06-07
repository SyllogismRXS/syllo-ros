function [X N Z] = control_p(t,x)
%==============================================================================
% Title  : Simple proportional controller
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
% Notes  : 
%==============================================================================
global depth_ref;
    
X = 0;
N = 0;
Z = 0;

depth_err = depth_ref - x(9);

Z = depth_err;

endfunction
