% function f_ext = ExternalForce(t, q,model)
% % F_pert 6x1 - roll, pitch, yaw, x,y,z
% F_pert =       [0 0 0            0 0 0]';
% 
% 
% % apply perturbation force on torso
% f_ext = cell(1,model.NB);
% bXw_curr = bodypos(model, model.idx.torso, q) ;
% f_ext{model.idx.torso} = bXw_curr' * F_pert;
function f_ext = ExternalForce(t, q,model)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
if t > 0.1
    F_pert =       [0 0 0            0 0 0]';
else
    F_pert =       [0 0 0            0 0 0]';
end

% Without being too specific on the test cases, if your controller works 
% for the following, it should do really well in terms of scoring: 
% Impulses of duration 0.1s or lesser with forces of magnitude 50% of M*g 
% or lower along translational directions and 10% of M*g or lower along 
% orientation directions.

% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;