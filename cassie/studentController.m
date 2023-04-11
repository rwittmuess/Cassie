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
    % kp = 500 ;
    % kd = 100 ;
    % x0 = getInitialState(model);
    % q0 = x0(1:model.n) ;
    % tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;

    %% [Control #4] Contact Force Optimization

    %%% Step 1: Object Force Generation, find Stabalizing Wrench %%%
    x0 = getInitialState(model); %48x1

    % compute current rotation
    Rx = rx(q(6)); % rotation around x-axis
    Ry = ry(q(5)); % rotation around y-axis
    Rz = rz(q(4)); % rotation around z-axis

    R = Rz * Ry * Rx; % maybe use this: function [phi,theta,psi] = RotToRPY_ZXY(R) (Libraries, spatial_v2_fea..., custom fcn

    % X = bodypos(model, model.idx.torso, q) ; 
    % R = X(1:3, 1:3)' ;
    Omega = bodyJac(model, model.idx.torso, q)*dq ;
    Omega = Omega(1:3);
    %Rd = eye(3);  Omd = zeros(3,1) ;



    % get current angular velocity Omega from state vector dq
    % entries 4,5,6 are yaw (z), pitch (y), roll (x)
%     Omega = [dq(6); dq(5); dq(4)];

    % J_b = bodyJac(model, model.idx.torso, q); % Libraries - spatial_v2_feath... - custom fcn
    % OmegaPreComp = J_b * dq;
    % Omega = OmegaPreComp(1:3);



    % x componennt is alreday in right frame, y and z need to be
    % transformed
    %RW: J = bodyJac(model, model.idx.torso,q)->Jdq, die ersten 3 sind
    %angular velocities

    % get current position of COM of upper body
%     r_c = [q(1); q(2); q(3)];
%     % get current velocity of COM of upper body
%     dr_c = [dq(1); dq(2); dq(3)];

    % get current position and velocity of COM (which is needed for F_GA)
    % [r_c, dr_c] = computeComPosVel(q, dq, model);
    r_c = compute_COM_pos(model, q) ;
    dr_c = COMJac_vel(model, q) * dq ;
    
    %given x0 and related COM position, and change desired position form that

    q_x0 = x0(1 : model.n);
    %dq_x0 = x0(model.n+1 : 2*model.n);
    %[r_c_d, dr_c_d] = computeComPosVel(q_x0, dq_x0, model);
    r_c_d = compute_COM_pos(model, q_x0) ;
    dr_c_d = zeros(3,1) ;

    g = 9.81;
    
    %% ---------------------- We need to tune those -----------------------

    Kp_f = 1000;
    Kd_f = 1000;
    Kp_t = 1000;
    Kd_t = 100;

    %% --------------------------------------------------------------------

    R_d = eye(3);
    R_hat = R_d'*R - R'*R_d;

    err_R = 0.5 * [R_hat(2,3); -R_hat(1,3); R_hat(1,2)];
    err_Omega = Omega; % - transpose(R) * R_d * Omega_d; %Omega_d is zero
    
    f_GA_d = -Kp_f * (r_c - r_c_d) - Kd_f * (dr_c - dr_c_d) + model.M*[0;0;g]; % + model.M*ddr_c_d; %ddr_c_d is zero
    tau_GA_d = R*(- Kp_t * err_R - Kd_t * err_Omega);% + eye(3) * dOmega_d; %dOmega_d is zero

    % tau is in body frame -> multuply with R to bribg it back to world
    % frame

    F_GA = [f_GA_d; tau_GA_d];
    % I tested F_GA for each component in the wrench individually by
    % applying an error term to the desired position / omega (s.o.)
    % changing the x-compionent only changes the x-component in the wrench ...
    % so only one entry is always non-zero, apart for z-component which
    % always has m*g


    %%% Step 2: Force Distribution, optimize Contact Forces that produce this desired wrench %%%
    
    % get global coordinates of contact of forces of the feet
    [p1, p2, p3, p4] = computeFootPositions(q, model);
    
    % use relative distance between COM and contact of forces of the feet
    % and apply the hat operator
    r_hat_1 = hatOperator(p1-r_c);
    r_hat_2 = hatOperator(p2-r_c);
    r_hat_3 = hatOperator(p3-r_c);
    r_hat_4 = hatOperator(p4-r_c);
    
    % compute the contact map correlating contact forces to the desired
    % wrench F_GA acting on COM
    G_C = [eye(3),  eye(3),  eye(3),  eye(3);
           r_hat_1, r_hat_2, r_hat_3, r_hat_4];

   
    % For testing purposes use the pseudoinverse: f_C = G_C_# * F_GA
    %G_C_pseu = pinv(G_C);
    f_C0 = pinv(G_C)*F_GA;

    % using the pseudoinverse I tested that the sum of the x,y,z components
    % of the contact forces always equal the x,y,z component of F_GA

    % use pseudo inverse before optimizarion to compute the contact forces
    % onyl if that works getting a suitabel tau, use the optimization
    % instead of the pseudoinverse (pinv function)


    % quadprog -> linear optimizer -> quadratic program, instead of fmincon
    alpha1 = 1;
    alpha2 = 1e-3;
    alpha3 = 1e-6;

    H = G_C(1:3,:)'*G_C(1:3,:) * (alpha1*2) + ...
        G_C(4:6,:)'*G_C(4:6,:) * (alpha2*2) + ...
        eye(12) * (alpha3*2);

    f = F_GA(1:3)'*G_C(1:3,:)*(-alpha1*2) - ...
        F_GA(4:6)'*G_C(4:6,:)*(-alpha2*2);

    mu = 0.8;
    [n1,n2,n3,n4] = fric_cone_norm_vec(mu);
    A = -[[n1;n2;n3;n4],zeros(4,9);
           zeros(4,3),[n1;n2;n3;n4],zeros(4,6);
           zeros(4,6),[n1;n2;n3;n4],zeros(4,3);
           zeros(4,9),[n1;n2;n3;n4]];
    b = zeros(16,1);

    Aeq = [];
    beq = [];
    lb = - [ 1000, 1000, 0, ...
             1000, 1000, 0, ...
             1000, 1000, 0, ...
             1000, 1000, 0]';
    ub = [   1000, 1000, 1000, ...
             1000, 1000, 1000, ...
             1000, 1000, 1000, ...
             1000, 1000, 1000]';

    options = optimset('Display', 'off');
    f_C = quadprog(H,f,A,b,Aeq,beq,lb,ub,f_C0,options);

    %%% Step 3: Force Mapping, convert Contact Foce into wrenches %%%

    % create wrenches for each contact point

    % get foot jacobians
    % originally 6x20, now 6x16, since the 4 columns coressponding to the
    % forces on the springs that we modeled to be constant
    [J1, J2, J3, J4] = computeFootJacobians(s,model);

    % convert wrenches to joint torques
    tau = -( J1'*[zeros(3,1); f_C(1:3)] + ...
             J2'*[zeros(3,1); f_C(4:6)] + ...
             J3'*[zeros(3,1); f_C(7:9)] + ...
             J4'*[zeros(3,1); f_C(10:12)] ) ;
    tau(1:6) = [] ; % First 6 joints are not actuated

end


%% Helper Functions for Controller
% Hat Operator
function r_hat = hatOperator(r)

    r_hat = [ 0,     r(3), -r(2);
             -r(3),  0,     r(1);
              r(2), -r(1),  0];
    
end

% to get normalized vectors as discussed on ED
% Friction Cone approximation
function [n1,n2,n3,n4] =  fric_cone_norm_vec(mu)

    xypart = mu*1/sqrt(2);
    v1 = [-xypart,-xypart,1];
    v2 = [+xypart,-xypart,1];
    v3 = [+xypart,+xypart,1];
    v4 = [-xypart,+xypart,1];
    n1 = v1/vecnorm(v1);
    n2 = v2/vecnorm(v2);
    n3 = v3/vecnorm(v3);
    n4 = v4/vecnorm(v4);

end