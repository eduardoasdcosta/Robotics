function [position, orientation] = direct_kinematics(dofs)

% Create link parameter vectors and create every transformation matrix
a = zeros(1,8);
alpha = zeros(1,8);
d = zeros(1,8);
teta = zeros(1,8);
T = zeros(4,4,8);

%% 
% Define parameters for each link

%link 1
a(1) = 0;
alpha(1) = 0;
d(1) = 99;
teta(1) = 0;

%link 2 - dof1
a(2) = 28;
alpha(2) = -90;
d(2) = 0;
teta(2) = 180 + dofs(1); 

%link 3 - dof2
a(3) = 120;
alpha(3) = 0;
d(3) = 0;
teta(3) = -90 + dofs(2); 

%link 4 - dof3
a(4) = 40;
alpha(4) = 0;
d(4) = 0;
teta(4) = -90 + dofs(3);

%link 5
a(5) = 19;
alpha(5) = -90;
d(5) = 0;
teta(5) = 90; 

%link 6 - dof4
a(6) = 0;
alpha(6) = 90;
d(6) = 167;
teta(6) = 0 + dofs(4); 

%link 7 - dof5
a(7) = 28;
alpha(7) = 0;
d(7) = 0;
teta(7) = 90  + dofs(5);

%link 8
a(8) = 0;
alpha(8) = 90;
d(8) = 0;
teta(8) = 90;

%%
% Compute transformation matrix between each joint and base-to-end transformation
for i = 1:8
    
    T(:,:,i) = create_trans_mat(d(i), 'Z')*create_rot_mat(teta(i), 'Z')*create_trans_mat(a(1,i), 'X')*create_rot_mat(alpha(i), 'X'); 
end

    T_Matrix = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7)*T(:,:,8)*create_rot_mat(dofs(6), 'Z');
    
% Compute position and orientation of end-effector 
    position = T_Matrix*[0 0 0 1]';
    position(4,:) = [];
    
    sbeta = -T_Matrix(3,1);
    cbeta = sqrt(T_Matrix(1,1)^2 + T_Matrix(2,1)^2);
    
    beta = atan2(sbeta, cbeta) * (180/pi);
    
    if beta == 90
        alfa = 0;
        gamma = atan2(T_Matrix(1,2), T_Matrix(2,2))  * (180/pi);
    elseif beta == -90
        alfa = 0;
        gamma = -atan2(T_Matrix(1,2), T_Matrix(2,2))  * (180/pi);
    else
       alfa = atan2(T_Matrix(2,1)/cbeta, T_Matrix(1,1)/cbeta)  * (180/pi);
       gamma = atan2(T_Matrix(3,2)/cbeta, T_Matrix(3,3)/cbeta)  * (180/pi);
    endif
    
    orientation = [alfa, beta, gamma];   
end

