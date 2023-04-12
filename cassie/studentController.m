function tau = studentController(t, s, model, params)
    q = s(1 : model.n);
    dq = s(model.n+1 : 2*model.n);
    
    X = bodypos(model, model.idx.torso, q) ; 
    R = X(1:3, 1:3)' ;
    Omega = bodyJac(model, model.idx.torso, q)*dq ;
    Omega = Omega(1:3);

    [r_c, dr_c] = computeComPosVel(q, dq, model);
	r_c_d = [-0.0103; 0; 0.8894];
    dr_c_d = - dr_c;
    
    Omega_d = -Omega;
    g = 9.81;

    kh = 40000;                     % tune this
    kv = 14000;                     % tune this
    dh = sqrt(model.M*kh)*2*0.8;
    dv = sqrt(model.M*kv)*2*0.2;
    
    Kp = diag([kh,kh,kv]);
    Kd = diag([dh,dh,dv]);
    
    Kr = eye(3)*100;                % tune this
    Dr = eye(3)*50;                 % tune this
    
    R_d = eye(3);
    err_R = 0.5 * skew(R_d'*R - R'*R_d);
    err_Omega = Omega - transpose(R) * R_d * Omega_d;
    f_GA_d = -Kp * (r_c - r_c_d) - Kd * (dr_c - dr_c_d) + model.M*[0;0;g];
    tau_GA_d = - Kr * err_R - Dr * err_Omega;

    F_GA = [f_GA_d; R*tau_GA_d];
    
    [p1, p2, p3, p4] = computeFootPositions(q, model);

    r_hat_1 = skew(p1-r_c);
    r_hat_2 = skew(p2-r_c);
    r_hat_3 = skew(p3-r_c);
    r_hat_4 = skew(p4-r_c);
    
    G_C = [eye(3),  eye(3),  eye(3),  eye(3);
           r_hat_1, r_hat_2, r_hat_3, r_hat_4];
    
    alpha1 = 1;
    alpha2 = 1e-3;
    alpha3 = 1e-6;
    G_C1= [eye(3) zeros(3)]*G_C;
    G_C2= [zeros(3) eye(3)]*G_C;
    H = G_C1'*G_C1 * (alpha1*2) + G_C2'*G_C2 * (alpha2*2) + eye(12) * (alpha3*2);
    f = F_GA(1:3)'*G_C1*(-alpha1*2) + F_GA(4:6)'*G_C2*(-alpha2*2);

    n= [-0.4417, -0.4417, 0.7809; ...
         0.4417, -0.4417, 0.7809; ...
         0.4417,  0.4417, 0.7809; ...
        -0.4417,  0.4417, 0.7809];
    
    A = zeros(4,12);
    A(sub2ind(size(A),1:4,3:3:12)) = -1;
    A = [A; -blkdiag(n,n,n,n)];
    b  = zeros(20,1);
    
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
%     lb = - [ 1000, 1000, 0, ...
%              1000, 1000, 0, ...
%              1000, 1000, 0, ...
%              1000, 1000, 0]';
%     ub = [   1000, 1000, 1000, ...
%              1000, 1000, 1000, ...
%              1000, 1000, 1000, ...
%              1000, 1000, 1000]';

    options = optimset('Display', 'off');
    f_C0 = [];
    % f_C0 = pinv(G_C)*F_GA;
    f_C = quadprog(H,f,A,b,Aeq,beq,lb,ub,f_C0,options);

    [J1, J2, J3, J4] = computeFootJacobians(s,model);
    tau = -( J1'*[zeros(3,1); f_C(1:3)] + ...
             J2'*[zeros(3,1); f_C(4:6)] + ...
             J3'*[zeros(3,1); f_C(7:9)] + ...
             J4'*[zeros(3,1); f_C(10:12)] ) ;
    tau(1:6) = [] ; % First 6 joints are not actuated
end