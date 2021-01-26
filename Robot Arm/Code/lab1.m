% 1st Lab Assignment - Direct and Inverse Kinematics of Serial Manipulators
% 
% Authors:
%     - Eduardo Costa  n. 84037
%     - Eduardo Melo   n. 84038
%     - Joao Sebastiao n. 84087

clear;
clc;

printf ("Inputs: \n");
%input the desired dofs for direct kinematics
direct_dofs = [0,0,0,0,0,0]

%input the desired position and orientation for inverse kinematics
inverse_pos = [250, 0, 150]
inverse_ori = [0, 0, 180]

% Direct Kinematics
% 
%     Input: degrees-of-freedom (dofs)
%     Outputs: position, orientation

[position, orientation] = direct_kinematics(direct_dofs);

printf ("\n\nDirect Kinematics outputs: \n");
printf ("\nPosition X: %d\nPosition Y: %d\nPosition Z: %d\n", position(1,1), position(2,1), position(3,1));
printf ("\nOrientation Alpha (Z): %d\nOrientation Beta  (Y): %d\nOrientation Gamma (X): %d\n\n", orientation(1,1), orientation(1,2), orientation(1,3));


% Inverse Kinematics
% 
%     Input: position, orientation
%     Outputs: degrees-of-freedom (dofs)


JointAngles = inverse_kinematics(inverse_pos, inverse_ori);
Row_number = rows(JointAngles);

printf ("\n\nInverse Kinematics outputs:");
if Row_number == 1
  printf ("\nInvalid position.\n");
else
  printf ("\n\n               Teta1      Teta2      Teta3      Teta4      Teta5      Teta6\n\n");
  for i = 1:Row_number
    
    printf ("Solution %d:  %7.2f    %7.2f    %7.2f    %7.2f    %7.2f    %7.2f\n", 
    i, JointAngles(i,1), JointAngles(i,2), JointAngles(i,3), JointAngles(i,4), JointAngles(i,5), JointAngles(i,6));
  end 
endif
  