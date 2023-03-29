% Modify this code to calculate the joint torques
% Input Parameters
%   t - time
%   s - state of the robot
%   model - struct containing robot properties
%   ctrl - any user defined control parameters in student_setup.m
% Output
%   tau - 10x1 vector of joint torques
function tau = studentController(t, s, model, params)

    %% Extract generalized coordinates and velocities
    q = s(1 : model.n);
    dq = s(model.n+1 : 2*model.n);

    %% [Control #1] zero control
    % tau = zeros(10,1);

    %% [Control #2] High Gain Joint PD control on all actuated joints
    kp = 500 ;
    kd = 100 ;
    x0 = getInitialState(model);
    q0 = x0(1:model.n) ;
    tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

end