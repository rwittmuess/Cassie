% % % % % % Modify this code to calculate the joint torques
% % % % % % Input Parameters
% % % % % %   t - time
% % % % % %   s - state of the robot
% % % % % %   model - struct containing robot properties
% % % % % %   ctrl - any user defined control parameters in student_setup.m
% % % % % % Output
% % % % % %   tau - 10x1 vector of joint torques
% % % % % function tau = studentController(t, s, model, params)
% % % % % 
% % % % %     %% Extract generalized coordinates and velocities
% % % % %     q = s(1 : model.n);
% % % % %     dq = s(model.n+1 : 2*model.n);
% % % % % 
% % % % %     %% [Control #1] zero control
% % % % %     % tau = zeros(10,1);
% % % % % 
% % % % %     %% [Control #2] High Gain Joint PD control on all actuated joints
% % % % %     % kp = 500 ;
% % % % %     % kd = 610 ; % kd = 100;
% % % % %     % x0 = getInitialState(model);
% % % % %     % q0 = x0(1:model.n) ;
% % % % %     % tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx);
% % % % % 
% % % % %     %% [Control #3] Posture and Balance Control based on Contact Force Optimization
% % % % % 
% % % % %     % %%% Richard trying stuff
% % % % %     % [r_c, v_c] = computeComPosVel(q, dq, model);
% % % % %     % [p1, p2, p3, p4] = computeFootPositions(q, model);
% % % % %     % r_i = [p1-r_c; p2-r_c; p3-r_c; p4-r_c];
% % % % %     % % calculating G_C and pseudoinverse
% % % % %     % r_hat_1 = [0 r_i(3)  -r_i(2);  -r_i(3)  0 r_i(1);  r_i(2)  -r_i(1),  0];
% % % % %     % r_hat_2 = [0 r_i(6)  -r_i(5);  -r_i(6)  0 r_i(4);  r_i(5)  -r_i(4),  0];
% % % % %     % r_hat_3 = [0 r_i(9)  -r_i(8);  -r_i(9)  0 r_i(7);  r_i(8)  -r_i(7),  0];
% % % % %     % r_hat_4 = [0 r_i(12) -r_i(11); -r_i(12) 0 r_i(10); r_i(11) -r_i(10), 0];
% % % % %     % G_C = cat(1,cat(2,eye(3),eye(3),eye(3),eye(3)),cat(2,r_hat_1,r_hat_2,r_hat_3,r_hat_4));
% % % % %     % % maybe we should use \ instead of inv()
% % % % %     % G_C_pseudoinverse = (G_C')*inv(G_C*(G_C')) % pinv(G_C)
% % % % % 
% % % % %     %% Step 1: Object Force Generation, find Stabalizing Wrench
% % % % %     x0 = getInitialState(model); %48x1
% % % % % 
% % % % %     % compute current rotation
% % % % %     Rx = rx(q(6)); % rotation around x-axis
% % % % %     Ry = ry(q(5)); % rotation around y-axis
% % % % %     Rz = rz(q(4)); % rotation around z-axis
% % % % % 
% % % % %     R = Rz * Ry * Rx; % maybe use this: function [phi,theta,psi] = RotToRPY_ZXY(R) (Libraries, spatial_v2_fea..., custom fcn
% % % % % 
% % % % %     % get current angular velocity Omega from state vector dq
% % % % %     % entries 4,5,6 are yaw (z), pitch (y), roll (x)
% % % % % %     Omega = [dq(6); dq(5); dq(4)];
% % % % % 
% % % % %     J_b = bodyJac(model, model.idx.torso, q); % Libraries - spatial_v2_feath... - custom fcn
% % % % %     preComp = J_b * dq;
% % % % %     Omega = preComp(1:3);
% % % % %     % x componennt is alreday in right frame, y and z need to be
% % % % %     % transformed
% % % % %     %RW: J = bodyJac(model, model.idx.torso,q)->Jdq, die ersten 3 sind
% % % % %     %angular velocities
% % % % % 
% % % % %     % get current position of COM of upper body
% % % % % %     r_c = [q(1); q(2); q(3)];
% % % % % %     % get current velocity of COM of upper body
% % % % % %     dr_c = [dq(1); dq(2); dq(3)];
% % % % % 
% % % % %     % get current position and velocity of COM (which is needed for F_GA)
% % % % %     [r_c, dr_c] = computeComPosVel(q, dq, model);
% % % % % 
% % % % %     %given x0 and related COM position, and change desired position form that
% % % % % 
% % % % %     q_x0 = x0(1 : model.n);
% % % % %     dq_x0 = x0(model.n+1 : 2*model.n);
% % % % %     [r_c_d, dr_c_d] = computeComPosVel(q_x0, dq_x0, model);
% % % % % 
% % % % %     g = 9.81;
% % % % %     Kp_f = 50;
% % % % %     Kd_f = 10;
% % % % %     Kp_t = 5;
% % % % %     Kd_t = 1;
% % % % %     r_c_d = r_c_d;% - [0;0;0.5];
% % % % %     dr_c_d = [0; 0; 0];
% % % % %     ddr_c_d = [0; 0; 0];
% % % % %     R_d = eye(3);
% % % % %     Omega_d = [0; 0; 0];%Omega; %- [0;0;0.5]; %[0; 0; 0];
% % % % %     dOmega_d = [0; 0; 0];
% % % % % 
% % % % %     R_hat = transpose(R_d) * R - transpose(R) * R_d;
% % % % % 
% % % % %     err_R = 0.5 * [R_hat(2,3); -R_hat(1,3); R_hat(1,2)];
% % % % % 
% % % % %     err_Omega = Omega - transpose(R) * R_d * Omega_d;
% % % % % 
% % % % %     f_GA_d = -Kp_f * (r_c - r_c_d) - Kd_f * (dr_c - dr_c_d) + model.M*[0;0;g] + model.M*ddr_c_d;
% % % % %     tau_GA_d = - Kp_t * err_R - Kd_t * err_Omega + eye(3) * dOmega_d;
% % % % % 
% % % % %     F_GA = [f_GA_d; tau_GA_d];
% % % % %     % I tested F_GA for each component in the wrench individually by
% % % % %     % applying an error term to the desired position / omega (s.o.)
% % % % %     % changing the x-compionent only changes the x-component in the wrench ...
% % % % %     % so only one entry is always non-zero, apart for z-component which
% % % % %     % always has m*g
% % % % % 
% % % % % 
% % % % %     %% Step 2: Force Distribution, optimize Contact Forces that produce this desired wrench
% % % % % 
% % % % %     % get global coordinates of contact of forces of the feet
% % % % %     [p1, p2, p3, p4] = computeFootPositions(q, model);
% % % % % 
% % % % %     % use relative distance between COM and contact of forces of the feet
% % % % %     % and apply the hat operator
% % % % % 
% % % % %     r_hat_1 = hatOperator(p1-r_c);
% % % % %     r_hat_2 = hatOperator(p2-r_c);
% % % % %     r_hat_3 = hatOperator(p3-r_c);
% % % % %     r_hat_4 = hatOperator(p4-r_c);
% % % % % 
% % % % %     % compute the contact map correlating contact forces to the desired
% % % % %     % wrench F_GA acting on COM
% % % % % %     G_C = cat(1,cat(2,eye(3),eye(3),eye(3),eye(3)),cat(2,r_hat_1,r_hat_2,r_hat_3,r_hat_4));
% % % % %     G_C = [eye(3),  eye(3),  eye(3),  eye(3);
% % % % %            r_hat_1, r_hat_2, r_hat_3, r_hat_4];
% % % % % 
% % % % % %     % first guess of an optimal solution for the optimization
% % % % % %     f_Copt0 = ones(4*3,1);
% % % % % %     
% % % % % %     % unilateral constraint (f_zi >= 0)
% % % % % %     % Ax <= b
% % % % % %     Aineq = [0,0,-1, 0,0,0, 0,0,0, 0,0,0;
% % % % % %              0,0,0, 0,0,-1, 0,0,0, 0,0,0;
% % % % % %              0,0,0, 0,0,0, 0,0,-1, 0,0,0;
% % % % % %              0,0,0, 0,0,0, 0,0,0, 0,0,-1]; 
% % % % % %     Bineq = zeros(4,1);
% % % % % % 
% % % % % %     Aeq = []; Beq = []; % we have no equality constraints
% % % % % % 
% % % % % %     % lower and upper bound on the decision variable f_c
% % % % % %     LB      = [-Inf, -Inf, -Inf, ... 
% % % % % %                -Inf, -Inf, -Inf, ...
% % % % % %                -Inf, -Inf, -Inf, ...
% % % % % %                -Inf, -Inf, -Inf]; 
% % % % % %     UB      = [Inf, Inf, Inf, ... 
% % % % % %                Inf, Inf, Inf, ...
% % % % % %                Inf, Inf, Inf, ...
% % % % % %                Inf, Inf, Inf];
% % % % % %     % 'display','iter',
% % % % % %     options = optimset('diffmaxchange',1.1*1e-5,'diffminchange',1e-5,'MaxFunEvals',20000,'MaxIter',20000,'TolCon',0.05);
% % % % % %     % options = optimset('display','iter','MaxFunEvals',20000,'MaxIter',20000);
% % % % % %     
% % % % % %     [f_C,obj_optim] = ...
% % % % % %         fmincon(@obj_cassie,f_Copt0,Aineq,Bineq,Aeq,Beq,LB,UB,@cons_cassie,options,F_GA,G_C);
% % % % % % 
% % % % % 
% % % % % 
% % % % %     % quadprog -> linear optimizer -> quadratic program, instead of fmincon
% % % % % 
% % % % % 
% % % % %     % For testing purposes use the pseudoinverse: f_C = G_C_# * F_GA
% % % % %     % G_C_pseu = pinv(G_C);
% % % % %     % f_C = G_C_pseu * F_GA;
% % % % %     % 
% % % % %     % f_x_sum = f_C(1) + f_C(4) + f_C(7) + f_C(10) - F_GA(1);
% % % % %     % f_y_sum = f_C(2) + f_C(5) + f_C(8) + f_C(11) - F_GA(2);
% % % % %     % f_z_sum = f_C(3) + f_C(6) + f_C(9) + f_C(12) - F_GA(3);
% % % % %     % using the pseudoinverse I tested that the sum of the x,y,z components
% % % % %     % of the contact forces always equal the x,y,z component of F_GA
% % % % % 
% % % % %     % use pseudo inverse before optimizarion to compute the contact forces
% % % % %     % onyl if that works getting a suitabel tau, use the optimization
% % % % %     % instead of the pseudoinverse (pinv function)
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % %     % Richard
% % % % % 
% % % % %     alpha1 = 1;
% % % % %     alpha2 = 0.001;
% % % % %     alpha3 = 0.00001;
% % % % % 
% % % % %     % Maricies for simplification
% % % % %     G_C_1 = cat(2,cat(1,eye(3), zeros(3)), cat(1,zeros(3), zeros(3)))*G_C;    % or G_C_pseu?!?!?!?!
% % % % %     G_C_2 = cat(2,cat(1,zeros(3), zeros(3)), cat(1,zeros(3), eye(3)))*G_C;
% % % % %     F_GA_1 = cat(2,cat(1,eye(3), zeros(3)), cat(1,zeros(3), zeros(3)))*F_GA;
% % % % %     F_GA_2 = cat(2,cat(1,zeros(3), zeros(3)), cat(1,zeros(3), eye(3)))*F_GA;
% % % % % 
% % % % %     % calculate H from cost
% % % % %     H_1 = 2*(G_C_1'*G_C_1); 
% % % % %     H_2 = 2*(G_C_2'*G_C_2);
% % % % %     H_3 = 2*eye(12);
% % % % %     H = alpha1*H_1 + alpha2*H_2 + alpha3*H_3;
% % % % % 
% % % % %     % calculate f from cost
% % % % %     f_1 = (-2*(F_GA_1).'*G_C)';
% % % % %     f_2 = (-2*F_GA_2.'*G_C)';
% % % % %     f = alpha1*f_1 + alpha2*f_2;    
% % % % % 
% % % % %     [n1,n2,n3,n4] = fric_cone_norm_vec(0.8); % mu = ??!?!?!
% % % % %     A = -[[n1';n2';n3';n4'],zeros(4,9);
% % % % %            zeros(4,3),[n1';n2';n3';n4'],zeros(4,6);
% % % % %            zeros(4,6),[n1';n2';n3';n4'],zeros(4,3);
% % % % %            zeros(4,9),[n1';n2';n3';n4']]; % Kroneker product
% % % % %     b = zeros(16,1);
% % % % % 
% % % % %     Aeq = [];
% % % % %     beq = [];
% % % % %     lb = -Inf*ones(12,1);
% % % % %     ub = ones(12,1)*Inf;
% % % % %     % lb = [-700;-700;0;-700;-700;0;-700;-700;0;-700;-700;0];
% % % % %     % ub = [700;700;700;700;700;700;700;700;700;700;700;700];
% % % % % 
% % % % % 
% % % % %     %x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options)
% % % % %     options = optimset('Display', 'off');
% % % % %     f_C = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % %     %%% Step 3: Force Mapping, convert Contact Foce into wrenches %%%
% % % % % 
% % % % %     % create wrenches for each contact point
% % % % %     F_K1 = [f_C(1:3);zeros(3,1)];
% % % % %     F_K2 = [f_C(4:6);zeros(3,1)];
% % % % %     F_K3 = [f_C(7:9);zeros(3,1)];
% % % % %     F_K4 = [f_C(10:12);zeros(3,1)];
% % % % % 
% % % % %     % get foot jacobians
% % % % %     % originally 6x20, now 6x16, since the 4 columns coressponding to the
% % % % %     % forces on the springs that we modeled to be constant
% % % % %     [J1, J2, J3, J4] = computeFootJacobians(s,model);
% % % % % 
% % % % % 
% % % % %     % convert wrenches to joint torques
% % % % %     tau = - transpose(J1)*F_K1 - transpose(J2)*F_K2 - transpose(J3)*F_K3 - transpose(J4)*F_K4;
% % % % %     tau = tau(7:end);
% % % % % 
% % % % % 
% % % % % end
% % % % % 
% % % % % %% Helper Functions
% % % % % 
% % % % % function [n1,n2,n3,n4] =  fric_cone_norm_vec(mu)
% % % % % 
% % % % %     % to get normalized vectors as discussed on ED
% % % % %     xypart = mu*1/sqrt(2);
% % % % %     v1 = [-xypart;-xypart;1];
% % % % %     v2 = [+xypart;-xypart;1];
% % % % %     v3 = [+xypart;+xypart;1];
% % % % %     v4 = [-xypart;+xypart;1];
% % % % %     n1 = v1/vecnorm(v1);
% % % % %     n2 = v2/vecnorm(v2);
% % % % %     n3 = v3/vecnorm(v3);
% % % % %     n4 = v4/vecnorm(v4);
% % % % % 
% % % % % end
% % % % % 
% % % % % %% Hat Operator
% % % % % function r_hat = hatOperator(r)
% % % % % 
% % % % %     r_hat = [0, r(3), -r(2);
% % % % %              -r(3), 0, r(1);
% % % % %              r(2), -r(1), 0];
% % % % % 
% % % % % end
% % % % % 
% % % % % %% Constraints
% % % % % function [cineq,ceq] = cons_cassie(f_C,F_GA,G_C)
% % % % % 
% % % % %     mu = 0.8;
% % % % % 
% % % % %     % friction cone
% % % % %     cineq = [1/(mu * f_C(3)) * sqrt(f_C(1).^2 + f_C(2).^2);
% % % % %              1/(mu * f_C(6)) * sqrt(f_C(4).^2 + f_C(5).^2);
% % % % %              1/(mu * f_C(9)) * sqrt(f_C(7).^2 + f_C(8).^2);
% % % % %              1/(mu * f_C(12)) * sqrt(f_C(10).^2 + f_C(11).^2)];
% % % % %     % cineq <= 0
% % % % %     ceq = [];
% % % % % 
% % % % % end
% % % % % 
% % % % % %% Objective Function
% % % % % function J = obj_cassie(f_C,F_GA,G_C)
% % % % % 
% % % % %     % WE need to DESIGN those variables
% % % % %     alpha1 = 10000;
% % % % %     alpha2 = 100;
% % % % %     alpha3 = 1;
% % % % % 
% % % % %     % Vincent
% % % % %     % F_opt = F_GA - G_C*f_C;
% % % % %     % 
% % % % %     % J1 = norm(F_opt(1:3))^2;
% % % % %     % J2 = norm(F_opt(4:6))^2;
% % % % %     % J3 = dot(f_C,f_C);
% % % % %     % 
% % % % %     % J = alpha1 * J1 + alpha2 * J2 + alpha3 * J3;
% % % % % 
% % % % %     % Richard
% % % % %     % [numRows,numCols] = size(F_GA)
% % % % %     J1 = norm((cat(2,eye(3),zeros(3)))*(F_GA-G_C*f_C))^2;
% % % % %     J2 = norm((cat(2,zeros(3),eye(3)))*(F_GA-G_C*f_C))^2;
% % % % %     J3 = f_C'*f_C;
% % % % % 
% % % % %     J = alpha1 * J1 + alpha2 * J2 + alpha3 * J3;
% % % % % 
% % % % % end
% % % % % 


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
%     kp = 500 ;
%     kd = 100 ;
%     x0 = getInitialState(model);
%     q0 = x0(1:model.n) ;
%     tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx) ;
%     
    %% [Control #4] Contact Force Optimization

    %%% Step 1: Object Force Generation, find Stabalizing Wrench %%%
    x0 = getInitialState(model); %48x1

    % compute current rotation
    Rx = rx(q(6)); % rotation around x-axis
    Ry = ry(q(5)); % rotation around y-axis
    Rz = rz(q(4)); % rotation around z-axis
    
    R = Rz * Ry * Rx; % maybe use this: function [phi,theta,psi] = RotToRPY_ZXY(R) (Libraries, spatial_v2_fea..., custom fcn
    
    % get current angular velocity Omega from state vector dq
    % entries 4,5,6 are yaw (z), pitch (y), roll (x)
%     Omega = [dq(6); dq(5); dq(4)];

    J_b = bodyJac(model, model.idx.torso, q); % Libraries - spatial_v2_feath... - custom fcn
    OmegaPreComp = J_b * dq;
    Omega = OmegaPreComp(1:3);
    % x componennt is alreday in right frame, y and z need to be
    % transformed
    %RW: J = bodyJac(model, model.idx.torso,q)->Jdq, die ersten 3 sind
    %angular velocities

    % get current position of COM of upper body
%     r_c = [q(1); q(2); q(3)];
%     % get current velocity of COM of upper body
%     dr_c = [dq(1); dq(2); dq(3)];

    % get current position and velocity of COM (which is needed for F_GA)
    [r_c, dr_c] = computeComPosVel(q, dq, model);
    
    %given x0 and related COM position, and change desired position form that

    q_x0 = x0(1 : model.n);
    dq_x0 = x0(model.n+1 : 2*model.n);
    [r_c_d, dr_c_d] = computeComPosVel(q_x0, dq_x0, model);

    g = 9.81;
    Kp_f = 500;
    Kd_f = 100;
    Kp_t = 500;
    Kd_t = 100;
    r_c_d = r_c_d;% - [0;0;0.01]; %%%%%%%%
    dr_c_d = [0; 0; 0];
    ddr_c_d = [0; 0; 0];
    R_d = eye(3);
    Omega_d = [0; 0; 0];%Omega; %- [0;0;0.5]; %[0; 0; 0];
    dOmega_d = [0; 0; 0];

    R_hat = transpose(R_d) * R - transpose(R) * R_d;

    err_R = 0.5 * [R_hat(2,3); -R_hat(1,3); R_hat(1,2)];
    err_Omega = Omega; % - transpose(R) * R_d * Omega_d; %Omega_d is zero
    
    f_GA_d = -Kp_f * (r_c - r_c_d) - Kd_f * (dr_c - dr_c_d) + model.M*[0;0;g]; % + model.M*ddr_c_d; %ddr_c_d is zero
    tau_GA_d = R\(- Kp_t * err_R - Kd_t * err_Omega);% + eye(3) * dOmega_d; %dOmega_d is zero

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
%     G_C = cat(1,cat(2,eye(3),eye(3),eye(3),eye(3)),cat(2,r_hat_1,r_hat_2,r_hat_3,r_hat_4));
    G_C = [eye(3),  eye(3),  eye(3),  eye(3);
           r_hat_1, r_hat_2, r_hat_3, r_hat_4];

   
    % For testing purposes use the pseudoinverse: f_C = G_C_# * F_GA
    G_C_pseu = pinv(G_C);
    f_C0 = G_C_pseu * F_GA;
    % 
    % f_x_sum = f_C0(1) + f_C0(4) + f_C0(7) + f_C0(10) - F_GA(1);
    % f_y_sum = f_C0(2) + f_C0(5) + f_C0(8) + f_C0(11) - F_GA(2);
    % f_z_sum = f_C0(3) + f_C0(6) + f_C0(9) + f_C0(12) - F_GA(3);
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
        G_C(4:6,:)'*G_C(4:6,:) * (alpha2*2)+ ...
        eye(12) * (alpha3*2);

    f = F_GA(1:3)'*G_C(1:3,:)*(-alpha1*2) - ...
        F_GA(4:6)'*G_C(4:6,:)*(-alpha2*2);

    mu = 0.8;
    [n1,n2,n3,n4] = fric_cone_norm_vec(mu);
    A = -[[n1;n2;n3;n4],zeros(4,9);
           zeros(4,3),[n1;n2;n3;n4],zeros(4,6);
           zeros(4,6),[n1;n2;n3;n4],zeros(4,3);
           zeros(4,9),[n1;n2;n3;n4]]; % Kroneker product
    b = zeros(16,1);

    Aeq = [];
    beq = [];
    lb = -Inf*ones(12,1);
    ub = ones(12,1)*Inf;

    options = optimset('Display', 'off');
    f_C = quadprog(H,f,A,b,Aeq,beq,lb,ub,f_C0,options);

    
    % Test if friction constraint satisfied, must be positive
    % TestFr10 = (sqrt(f_C0(1)^2+f_C0(2)^2) / f_C0(3)) - mu;
    % TestFr20 = (sqrt(f_C0(4)^2+f_C0(5)^2) / f_C0(6)) - mu;
    % TestFr30 = (sqrt(f_C0(7)^2+f_C0(8)^2) / f_C0(9)) - mu;
    % 
    % TestFr1 = (sqrt(f_C(1)^2+f_C(2)^2) / f_C(3)) - mu;
    % TestFr2 = (sqrt(f_C(4)^2+f_C(5)^2) / f_C(6)) - mu;
    % TestFr3 = (sqrt(f_C(7)^2+f_C(8)^2) / f_C(9)) - mu;



    %%% Step 3: Force Mapping, convert Contact Foce into wrenches %%%

    % create wrenches for each contact point
    % the libraries expect tau first and then forces
    F_K1 = [zeros(3,1);f_C(1:3)];
    F_K2 = [zeros(3,1);f_C(4:6)];
    F_K3 = [zeros(3,1);f_C(7:9)];
    F_K4 = [zeros(3,1);f_C(10:12)];
    % switch around Tau first

    % get foot jacobians
    % originally 6x20, now 6x16, since the 4 columns coressponding to the
    % forces on the springs that we modeled to be constant
    [J1, J2, J3, J4] = computeFootJacobians(s,model);
    % J1 = J1';
    % J2 = J2';
    % J3 = J3';
    % J4 = J4';

    % convert wrenches to joint torques
    %tau = - transpose(J1)*F_K1 - transpose(J2)*F_K2 - transpose(J3)*F_K3 - transpose(J4)*F_K4;
    tau = -J1'*F_K1 -J2'*F_K2 -J3'*F_K3 -J4'*F_K4;
    % tau= -J1(4:6)'.*f_C(1:3) -J2(4:6)'.*f_C(4:6) -J3(4:6)'.*f_C(7:9) -J4(4:6)'.*f_C(10:12);
    % tau
    tau = tau(7:end);

end


%% Helper Functions for Controller
% Hat Operator
function r_hat = hatOperator(r)

    r_hat = [0,     r(3),  -r(2);
             -r(3), 0,     r(1);
             r(2),  -r(1), 0];
    
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