function JointAngles = inverse_kinematics(position, orientation)
%returns a matrix with 6 columns and 4 or 8 rows in case of solutions found,
%otherwise returns -1

T_mat = zeros(4);

%% 
% Define parameters for each link
d = [99 0 0 0 0];
teta = [0 0 -90 180 90];
a = [0 28 120 40 19];
alpha = [0 -90 0 0 -90];
teta(7) = -90; %angle that needs to be added to dof5
%the remaining values of teta are calculated and stored in JointAngles

% create Transformation Matrix (0_8)
rot = create_rot_mat(orientation(1),'Z')*create_rot_mat(orientation(2), 'Y')*create_rot_mat(orientation(3), 'X');
p8 = [position(1) position(2) position(3) 1]';
T_mat = rot;
T_mat(:, 4) = p8;

%Find Wrist point (Point of the 5th joint)
Wrist = T_mat*[0 0 -28 1]';

%get both possibilities for teta1
if Wrist(1) == 0  % if there is no component in x
   if Wrist(2) > 0
      Teta1A = 90;
   elseif Wrist(2) == 0 %if there is no component in x and y
      Teta1A = 0;
      printf ("\nInfinite solutions for teta1 and teta6.\n");
   else
      Teta1A = -90;
   endif
else
   Teta1A = atan2(Wrist(2), Wrist(1))*180/pi;
endif    

if(Teta1A <= 0)
   Teta1B = Teta1A + 180;
else
   Teta1B = Teta1A - 180;
endif

%transform base reference frame to joint 2 center and with rotation angle teta 1

ShoulderT1 = create_trans_mat(-28, 'X')*create_rot_mat(-Teta1A, 'Z')*create_trans_mat(-99, 'Z');
ShoulderT2 = create_trans_mat(-28, 'X')*create_rot_mat(-Teta1B, 'Z')*create_trans_mat(-99, 'Z');

% Wrist point in reference frame of joint 2

WristPoint1 = ShoulderT1*Wrist;
WristPoint2 = ShoulderT2*Wrist;

%distances used to compute teta3 and teta2 
a1 = 120;
a2 = sqrt(127^2+19^2);

%variables used to check if the distance is valid
Check1 = abs((WristPoint1(1)^2 + WristPoint1(3)^2 - a1^2 - a2^2) / (2 * a1 * a2));
Check2 = abs((WristPoint2(1)^2 + WristPoint2(3)^2 - a1^2 - a2^2) / (2 * a1 * a2));

if Check1 <= 1 || Check2 <= 1
  if Check1 <= 1    %if the position is valid for Teta1A, compute the correspondent 4 solutions
    JointAngles(1:4, 1) = Teta1A;
    
    %calculate both possibilities for Teta3
    JointAngles(1:2, 3) = acos((WristPoint1(1)^2 + WristPoint1(3)^2 - a1^2 - a2^2) / (2 * a1 * a2)) * 180/pi; 
    JointAngles(3:4, 3) = -JointAngles(1,3);

    %Calculate both possibilities for teta2 using geometry and "cosine law", being each teta2 correspondent to one of the teta3 values
    Phi = acos((WristPoint1(1)^2 + WristPoint1(3)^2 + a1^2 - a2^2) / (2 * a1 * sqrt(WristPoint1(1)^2 + WristPoint1(3)^2))) * 180/pi;
    Beta = atan2(WristPoint1(3), WristPoint1(1)) * 180/pi;

    JointAngles(1:2, 2) = Beta - Phi;
    JointAngles(3:4, 2) = Beta + Phi;  
  
    %compute the 5 transformation matrices for one of the values of teta2/teta3,
    %from which we already know/computed the values of teta (0-1, 1-2, 2-3, 3-4, 4-5)
    T0 = zeros(4);
    T1 = zeros(4);
    T2 = zeros(4);
    T3 = zeros(4);
    T4 = zeros(4);
    
    T0(1,1) = cosd(teta(1));
    T0(1,2) = -sind(teta(1)) * cosd(alpha(1));
    T0(1,3) = sind(teta(1)) * sind(alpha(1));
    T0(1,4) = a(1) * cosd(teta(1));
    T0(2,1) = sind(teta(1));
    T0(2,2) = cosd(teta(1)) * cosd(alpha(1));
    T0(2,3) = -cosd(teta(1)) * sind(alpha(1));
    T0(2,4) = a(1) * sind(teta(1));
    T0(3,2) = sind(alpha(1));
    T0(3,3) = cosd(alpha(1));
    T0(3,4) = d(1);
    T0(4,4) = 1;
    
    T1(1,1) = cosd(teta(2) + JointAngles(1, 1));
    T1(1,2) = -sind(teta(2) + JointAngles(1, 1)) * cosd(alpha(2));
    T1(1,3) = sind(teta(2) + JointAngles(1, 1)) * sind(alpha(2));
    T1(1,4) = a(2) * cosd(teta(2) + JointAngles(1, 1));
    T1(2,1) = sind(teta(2) + JointAngles(1, 1));
    T1(2,2) = cosd(teta(2) + JointAngles(1, 1)) * cosd(alpha(2));
    T1(2,3) = -cosd(teta(2) + JointAngles(1, 1)) * sind(alpha(2));
    T1(2,4) = a(2) * sind(teta(2) + JointAngles(1, 1));
    T1(3,2) = sind(alpha(2));
    T1(3,3) = cosd(alpha(2));
    T1(3,4) = d(2);
    T1(4,4) = 1;

    T2(1,1) = cosd(teta(3) + JointAngles(1, 2));
    T2(1,2) = -sind(teta(3) + JointAngles(1, 2)) * cosd(alpha(3));
    T2(1,3) = sind(teta(3) + JointAngles(1, 2)) * sind(alpha(3));
    T2(1,4) = a(3) * cosd(teta(3) + JointAngles(1, 2));
    T2(2,1) = sind(teta(3) + JointAngles(1, 2));
    T2(2,2) = cosd(teta(3) + JointAngles(1, 2)) * cosd(alpha(3));
    T2(2,3) = -cosd(teta(3) + JointAngles(1, 2)) * sind(alpha(3));
    T2(2,4) = a(3) * sind(teta(3) + JointAngles(1, 2));
    T2(3,2) = sind(alpha(3));
    T2(3,3) = cosd(alpha(3));
    T2(3,4) = d(3);
    T2(4,4) = 1;

    T3(1,1) = cosd(teta(4) + JointAngles(1, 3));
    T3(1,2) = -sind(teta(4) + JointAngles(1, 3)) * cosd(alpha(4));
    T3(1,3) = sind(teta(4) + JointAngles(1, 3)) * sind(alpha(4));
    T3(1,4) = a(4) * cosd(teta(4) + JointAngles(1, 3));
    T3(2,1) = sind(teta(4) + JointAngles(1, 3));
    T3(2,2) = cosd(teta(4) + JointAngles(1, 3)) * cosd(alpha(4));
    T3(2,3) = -cosd(teta(4) + JointAngles(1, 3)) * sind(alpha(4));
    T3(2,4) = a(4) * sind(teta(4) + JointAngles(1, 3));
    T3(3,2) = sind(alpha(4));
    T3(3,3) = cosd(alpha(4));
    T3(3,4) = d(4);
    T3(4,4) = 1;

    T4(1,1) = cosd(teta(5));
    T4(1,2) = -sind(teta(5)) * cosd(alpha(5));
    T4(1,3) = sind(teta(5)) * sind(alpha(5));
    T4(1,4) = a(5) * cosd(teta(5));
    T4(2,1) = sind(teta(5));
    T4(2,2) = cosd(teta(5)) * cosd(alpha(5));
    T4(2,3) = -cosd(teta(5)) * sind(alpha(5));
    T4(2,4) = a(5) * sind(teta(5));
    T4(3,2) = sind(alpha(5));
    T4(3,3) = cosd(alpha(5));
    T4(3,4) = d(5);
    T4(4,4) = 1;
    
    %compute the tranformation matrix (0-4) for one solution of teta2/teta3    
    TProduct1 = T0 * T1 * T2 * T3 * T4;
    
    %compute the 5 transformation matrices for one of the values of teta2/teta3,
    %from which we already know/computed the values of teta (0-1, 1-2, 2-3, 3-4, 4-5)
    T0 = zeros(4);
    T1 = zeros(4);
    T2 = zeros(4);
    T3 = zeros(4);
    T4 = zeros(4);
    
    T0(1,1) = cosd(teta(1));
    T0(1,2) = -sind(teta(1)) * cosd(alpha(1));
    T0(1,3) = sind(teta(1)) * sind(alpha(1));
    T0(1,4) = a(1) * cosd(teta(1));
    T0(2,1) = sind(teta(1));
    T0(2,2) = cosd(teta(1)) * cosd(alpha(1));
    T0(2,3) = -cosd(teta(1)) * sind(alpha(1));
    T0(2,4) = a(1) * sind(teta(1));
    T0(3,2) = sind(alpha(1));
    T0(3,3) = cosd(alpha(1));
    T0(3,4) = d(1);
    T0(4,4) = 1;

    T1(1,1) = cosd(teta(2) + JointAngles(3, 1));
    T1(1,2) = -sind(teta(2) + JointAngles(3, 1)) * cosd(alpha(2));
    T1(1,3) = sind(teta(2) + JointAngles(3, 1)) * sind(alpha(2));
    T1(1,4) = a(2) * cosd(teta(2) + JointAngles(3, 1));
    T1(2,1) = sind(teta(2) + JointAngles(3, 1));
    T1(2,2) = cosd(teta(2) + JointAngles(3, 1)) * cosd(alpha(2));
    T1(2,3) = -cosd(teta(2) + JointAngles(3, 1)) * sind(alpha(2));
    T1(2,4) = a(2) * sind(teta(2) + JointAngles(3, 1));
    T1(3,2) = sind(alpha(2));
    T1(3,3) = cosd(alpha(2));
    T1(3,4) = d(2);
    T1(4,4) = 1;

    T2(1,1) = cosd(teta(3) + JointAngles(3, 2));
    T2(1,2) = -sind(teta(3) + JointAngles(3, 2)) * cosd(alpha(3));
    T2(1,3) = sind(teta(3) + JointAngles(3, 2)) * sind(alpha(3));
    T2(1,4) = a(3) * cosd(teta(3) + JointAngles(3, 2));
    T2(2,1) = sind(teta(3) + JointAngles(3, 2));
    T2(2,2) = cosd(teta(3) + JointAngles(3, 2)) * cosd(alpha(3));
    T2(2,3) = -cosd(teta(3) + JointAngles(3, 2)) * sind(alpha(3));
    T2(2,4) = a(3) * sind(teta(3) + JointAngles(3, 2));
    T2(3,2) = sind(alpha(3));
    T2(3,3) = cosd(alpha(3));
    T2(3,4) = d(3);
    T2(4,4) = 1;

    T3(1,1) = cosd(teta(4) + JointAngles(3, 3));
    T3(1,2) = -sind(teta(4) + JointAngles(3, 3)) * cosd(alpha(4));
    T3(1,3) = sind(teta(4) + JointAngles(3, 3)) * sind(alpha(4));
    T3(1,4) = a(4) * cosd(teta(4) + JointAngles(3, 3));
    T3(2,1) = sind(teta(4) + JointAngles(3, 3));
    T3(2,2) = cosd(teta(4) + JointAngles(3, 3)) * cosd(alpha(4));
    T3(2,3) = -cosd(teta(4) + JointAngles(3, 3)) * sind(alpha(4));
    T3(2,4) = a(4) * sind(teta(4) + JointAngles(3, 3));
    T3(3,2) = sind(alpha(4));
    T3(3,3) = cosd(alpha(4));
    T3(3,4) = d(4);
    T3(4,4) = 1;

    T4(1,1) = cosd(teta(5));
    T4(1,2) = -sind(teta(5)) * cosd(alpha(5));
    T4(1,3) = sind(teta(5)) * sind(alpha(5));
    T4(1,4) = a(5) * cosd(teta(5));
    T4(2,1) = sind(teta(5));
    T4(2,2) = cosd(teta(5)) * cosd(alpha(5));
    T4(2,3) = -cosd(teta(5)) * sind(alpha(5));
    T4(2,4) = a(5) * sind(teta(5));
    T4(3,2) = sind(alpha(5));
    T4(3,3) = cosd(alpha(5));
    T4(3,4) = d(5);
    T4(4,4) = 1;
        
    %compute the tranformation matrix (0-4) for one solution of teta2/teta3 
    TProduct2 = T0 * T1 * T2 * T3 * T4;
    
    %compute the inverse of both transformation matrixes
    TProductRot1 = TProduct1(1:3, 1:3);
    TProductTrans1 = -TProductRot1' * TProduct1(1:3, 4);
    TProductTrans1(4) = 1;
    TProductInv1 = TProductRot1';
    TProductInv1(4, 1:3) = [0 0 0];
    TProductInv1(:,4) = TProductTrans1;

    TProductRot2 = TProduct2(1:3, 1:3);
    TProductTrans2 = -TProductRot2' * TProduct2(1:3, 4);
    TProductTrans2(4) = 1;
    TProductInv2 = TProductRot2';
    TProductInv2(4, 1:3) = [0 0 0];
    TProductInv2(:,4) = TProductTrans2;
    
    %compute the left hand matrices for both solutions    
    LeftHandMatrix1 = TProductInv1 * T_mat;
    LeftHandMatrix2 = TProductInv2 * T_mat;
    
    %compute the correspondent 4 solutions for teta4, teta5, teta6
    %correspondent to both values of teta2/teta3
    %and correspondent to one value of teta1
    JointAngles(1,5) = teta(7) + asin(LeftHandMatrix1(3,3)) * 180/pi;
    JointAngles(2,5) = -JointAngles(1,5);
    JointAngles(3,5) = teta(7) + asin(LeftHandMatrix2(3,3)) * 180/pi;
    JointAngles(4,5) = -JointAngles(3,5);
    
    if (cos(JointAngles(1,5)) == 0 || cos(JointAngles(3,5)) == 0)
        printf("Infinite solutions for teta4 and teta6.\n");
    endif

    JointAngles(1,6) = atan2(LeftHandMatrix1(3,2), LeftHandMatrix1(3,1)) * 180/pi;
    JointAngles(2,6) = atan2(-LeftHandMatrix1(3,2), -LeftHandMatrix1(3,1)) * 180/pi;
    JointAngles(3,6) = atan2(LeftHandMatrix2(3,2), LeftHandMatrix2(3,1)) * 180/pi;
    JointAngles(4,6) = atan2(-LeftHandMatrix2(3,2), -LeftHandMatrix2(3,1)) * 180/pi;

    JointAngles(1,4) = atan2(LeftHandMatrix1(2,3), LeftHandMatrix1(1,3)) * 180/pi;
    JointAngles(2,4) = atan2(-LeftHandMatrix1(2,3), -LeftHandMatrix1(1,3)) * 180/pi;
    JointAngles(3,4) = atan2(LeftHandMatrix2(2,3), LeftHandMatrix2(1,3)) * 180/pi;
    JointAngles(4,4) = atan2(-LeftHandMatrix2(2,3), -LeftHandMatrix2(1,3)) * 180/pi;   
  endif
  
  if Check2 <= 1 %if the position is valid for Teta1B, compute the correspondent 4 solutions
    JointAngles(5:8, 1) = Teta1B;
    
    %calculate both possibilities for Teta3
    JointAngles(5:6, 3) = acos((WristPoint2(1)^2 + WristPoint2(3)^2 - a1^2 - a2^2) / (2 * a1 * a2)) * 180/pi;
    JointAngles(7:8, 3) = -JointAngles(5, 3);

    %Calculate both possibilities for teta2 using geometry and "cosine law", being each teta2 correspondent to one of the teta3 values
    Phi = acos((WristPoint2(1)^2 + WristPoint2(3)^2 + a1^2 - a2^2) / (2 * a1 * sqrt(WristPoint2(1)^2 + WristPoint2(3)^2))) * 180/pi;
    Beta = atan2(WristPoint2(3), WristPoint2(1)) * 180/pi;

    JointAngles(5:6, 2) = Beta - Phi;
    JointAngles(7:8, 2) = Beta + Phi;
    
    %compute the 5 transformation matrices for one of the values of teta2/teta3,
    %from which we already know/computed the values of teta (0-1, 1-2, 2-3, 3-4, 4-5)
    T0 = zeros(4);
    T1 = zeros(4);
    T2 = zeros(4);
    T3 = zeros(4);
    T4 = zeros(4);
    
    T0(1,1) = cosd(teta(1));
    T0(1,2) = -sind(teta(1)) * cosd(alpha(1));
    T0(1,3) = sind(teta(1)) * sind(alpha(1));
    T0(1,4) = a(1) * cosd(teta(1));
    T0(2,1) = sind(teta(1));
    T0(2,2) = cosd(teta(1)) * cosd(alpha(1));
    T0(2,3) = -cosd(teta(1)) * sind(alpha(1));
    T0(2,4) = a(1) * sind(teta(1));
    T0(3,2) = sind(alpha(1));
    T0(3,3) = cosd(alpha(1));
    T0(3,4) = d(1);
    T0(4,4) = 1;

    T1(1,1) = cosd(teta(2) + JointAngles(5, 1));
    T1(1,2) = -sind(teta(2) + JointAngles(5, 1)) * cosd(alpha(2));
    T1(1,3) = sind(teta(2) + JointAngles(5, 1)) * sind(alpha(2));
    T1(1,4) = a(2) * cosd(teta(2) + JointAngles(5, 1));
    T1(2,1) = sind(teta(2) + JointAngles(5, 1));
    T1(2,2) = cosd(teta(2) + JointAngles(5, 1)) * cosd(alpha(2));
    T1(2,3) = -cosd(teta(2) + JointAngles(5, 1)) * sind(alpha(2));
    T1(2,4) = a(2) * sind(teta(2) + JointAngles(5, 1));
    T1(3,2) = sind(alpha(2));
    T1(3,3) = cosd(alpha(2));
    T1(3,4) = d(2);
    T1(4,4) = 1;

    T2(1,1) = cosd(teta(3) + JointAngles(5, 2));
    T2(1,2) = -sind(teta(3) + JointAngles(5, 2)) * cosd(alpha(3));
    T2(1,3) = sind(teta(3) + JointAngles(5, 2)) * sind(alpha(3));
    T2(1,4) = a(3) * cosd(teta(3) + JointAngles(5, 2));
    T2(2,1) = sind(teta(3) + JointAngles(5, 2));
    T2(2,2) = cosd(teta(3) + JointAngles(5, 2)) * cosd(alpha(3));
    T2(2,3) = -cosd(teta(3) + JointAngles(5, 2)) * sind(alpha(3));
    T2(2,4) = a(3) * sind(teta(3) + JointAngles(5, 2));
    T2(3,2) = sind(alpha(3));
    T2(3,3) = cosd(alpha(3));
    T2(3,4) = d(3);
    T2(4,4) = 1;

    T3(1,1) = cosd(teta(4) + JointAngles(5, 3));
    T3(1,2) = -sind(teta(4) + JointAngles(5, 3)) * cosd(alpha(4));
    T3(1,3) = sind(teta(4) + JointAngles(5, 3)) * sind(alpha(4));
    T3(1,4) = a(4) * cosd(teta(4) + JointAngles(5, 3));
    T3(2,1) = sind(teta(4) + JointAngles(5, 3));
    T3(2,2) = cosd(teta(4) + JointAngles(5, 3)) * cosd(alpha(4));
    T3(2,3) = -cosd(teta(4) + JointAngles(5, 3)) * sind(alpha(4));
    T3(2,4) = a(4) * sind(teta(4) + JointAngles(5, 3));
    T3(3,2) = sind(alpha(4));
    T3(3,3) = cosd(alpha(4));
    T3(3,4) = d(4);
    T3(4,4) = 1;

    T4(1,1) = cosd(teta(5));
    T4(1,2) = -sind(teta(5)) * cosd(alpha(5));
    T4(1,3) = sind(teta(5)) * sind(alpha(5));
    T4(1,4) = a(5) * cosd(teta(5));
    T4(2,1) = sind(teta(5));
    T4(2,2) = cosd(teta(5)) * cosd(alpha(5));
    T4(2,3) = -cosd(teta(5)) * sind(alpha(5));
    T4(2,4) = a(5) * sind(teta(5));
    T4(3,2) = sind(alpha(5));
    T4(3,3) = cosd(alpha(5));
    T4(3,4) = d(5);
    T4(4,4) = 1;
    
    %compute the tranformation matrix (0-4) for one solution of teta2/teta3  
    TProduct1 = T0 * T1 * T2 * T3 * T4;
    
    %compute the 5 transformation matrices for one of the values of teta2/teta3,
    %from which we already know/computed the values of teta (0-1, 1-2, 2-3, 3-4, 4-5)
    T0 = zeros(4);
    T1 = zeros(4);
    T2 = zeros(4);
    T3 = zeros(4);
    T4 = zeros(4);
    
    T0(1,1) = cosd(teta(1));
    T0(1,2) = -sind(teta(1)) * cosd(alpha(1));
    T0(1,3) = sind(teta(1)) * sind(alpha(1));
    T0(1,4) = a(1) * cosd(teta(1));
    T0(2,1) = sind(teta(1));
    T0(2,2) = cosd(teta(1)) * cosd(alpha(1));
    T0(2,3) = -cosd(teta(1)) * sind(alpha(1));
    T0(2,4) = a(1) * sind(teta(1));
    T0(3,2) = sind(alpha(1));
    T0(3,3) = cosd(alpha(1));
    T0(3,4) = d(1);
    T0(4,4) = 1;

    T1(1,1) = cosd(teta(2) + JointAngles(7, 1));
    T1(1,2) = -sind(teta(2) + JointAngles(7, 1)) * cosd(alpha(2));
    T1(1,3) = sind(teta(2) + JointAngles(7, 1)) * sind(alpha(2));
    T1(1,4) = a(2) * cosd(teta(2) + JointAngles(7, 1));
    T1(2,1) = sind(teta(2) + JointAngles(7, 1));
    T1(2,2) = cosd(teta(2) + JointAngles(7, 1)) * cosd(alpha(2));
    T1(2,3) = -cosd(teta(2) + JointAngles(7, 1)) * sind(alpha(2));
    T1(2,4) = a(2) * sind(teta(2) + JointAngles(7, 1));
    T1(3,2) = sind(alpha(2));
    T1(3,3) = cosd(alpha(2));
    T1(3,4) = d(2);
    T1(4,4) = 1;

    T2(1,1) = cosd(teta(3) + JointAngles(7, 2));
    T2(1,2) = -sind(teta(3) + JointAngles(7, 2)) * cosd(alpha(3));
    T2(1,3) = sind(teta(3) + JointAngles(7, 2)) * sind(alpha(3));
    T2(1,4) = a(3) * cosd(teta(3) + JointAngles(7, 2));
    T2(2,1) = sind(teta(3) + JointAngles(7, 2));
    T2(2,2) = cosd(teta(3) + JointAngles(7, 2)) * cosd(alpha(3));
    T2(2,3) = -cosd(teta(3) + JointAngles(7, 2)) * sind(alpha(3));
    T2(2,4) = a(3) * sind(teta(3) + JointAngles(7, 2));
    T2(3,2) = sind(alpha(3));
    T2(3,3) = cosd(alpha(3));
    T2(3,4) = d(3);
    T2(4,4) = 1;

    T3(1,1) = cosd(teta(4) + JointAngles(7, 3));
    T3(1,2) = -sind(teta(4) + JointAngles(7, 3)) * cosd(alpha(4));
    T3(1,3) = sind(teta(4) + JointAngles(7, 3)) * sind(alpha(4));
    T3(1,4) = a(4) * cosd(teta(4) + JointAngles(7, 3));
    T3(2,1) = sind(teta(4) + JointAngles(7, 3));
    T3(2,2) = cosd(teta(4) + JointAngles(7, 3)) * cosd(alpha(4));
    T3(2,3) = -cosd(teta(4) + JointAngles(7, 3)) * sind(alpha(4));
    T3(2,4) = a(4) * sind(teta(4) + JointAngles(7, 3));
    T3(3,2) = sind(alpha(4));
    T3(3,3) = cosd(alpha(4));
    T3(3,4) = d(4);
    T3(4,4) = 1;

    T4(1,1) = cosd(teta(5));
    T4(1,2) = -sind(teta(5)) * cosd(alpha(5));
    T4(1,3) = sind(teta(5)) * sind(alpha(5));
    T4(1,4) = a(5) * cosd(teta(5));
    T4(2,1) = sind(teta(5));
    T4(2,2) = cosd(teta(5)) * cosd(alpha(5));
    T4(2,3) = -cosd(teta(5)) * sind(alpha(5));
    T4(2,4) = a(5) * sind(teta(5));
    T4(3,2) = sind(alpha(5));
    T4(3,3) = cosd(alpha(5));
    T4(3,4) = d(5);
    T4(4,4) = 1;
        
    %compute the tranformation matrix (0-4) for one solution of teta2/teta3
    TProduct2 = T0 * T1 * T2 * T3 * T4;
    
    %compute the inverse of both transformation matrixes
    TProductRot1 = TProduct1(1:3, 1:3);
    TProductTrans1 = -TProductRot1' * TProduct1(1:3, 4);
    TProductTrans1(4) = 1;
    TProductInv1 = TProductRot1';
    TProductInv1(4, 1:3) = [0 0 0];
    TProductInv1(:,4) = TProductTrans1;

    TProductRot2 = TProduct2(1:3, 1:3);
    TProductTrans2 = -TProductRot2' * TProduct2(1:3, 4);
    TProductTrans2(4) = 1;
    TProductInv2 = TProductRot2';
    TProductInv2(4, 1:3) = [0 0 0];
    TProductInv2(:,4) = TProductTrans2;
    
        
    %compute the left hand matrices for both solutions 
    LeftHandMatrix1 = TProductInv1 * T_mat;
    LeftHandMatrix2 = TProductInv2 * T_mat;
    
    %compute the correspondent 4 solutions for teta4, teta5, teta6
    %correspondent to both values of teta2/teta3
    %and correspondent to one value of teta1
    JointAngles(5,5) = teta(7) + asin(LeftHandMatrix1(3,3)) * 180/pi;
    JointAngles(6,5) = -JointAngles(5,5);
    JointAngles(7,5) = teta(7) + asin(LeftHandMatrix2(3,3)) * 180/pi;
    JointAngles(8,5) = -JointAngles(7,5);
    
    if (cos(JointAngles(5,5)) == 0 || cos(JointAngles(7,5)) == 0)
          printf("Infinite solutions for teta4 and teta6.\n");
    endif

    JointAngles(5,6) = atan2(LeftHandMatrix1(3,2), LeftHandMatrix1(3,1)) * 180/pi;
    JointAngles(6,6) = atan2(-LeftHandMatrix1(3,2), -LeftHandMatrix1(3,1)) * 180/pi;
    JointAngles(7,6) = atan2(LeftHandMatrix2(3,2), LeftHandMatrix2(3,1)) * 180/pi;
    JointAngles(8,6) = atan2(-LeftHandMatrix2(3,2), -LeftHandMatrix2(3,1)) * 180/pi;

    JointAngles(5,4) = atan2(LeftHandMatrix1(2,3), LeftHandMatrix1(1,3)) * 180/pi;
    JointAngles(6,4) = atan2(-LeftHandMatrix1(2,3), -LeftHandMatrix1(1,3)) * 180/pi;
    JointAngles(7,4) = atan2(LeftHandMatrix2(2,3), LeftHandMatrix2(1,3)) * 180/pi;
    JointAngles(8,4) = atan2(-LeftHandMatrix2(2,3), -LeftHandMatrix2(1,3)) * 180/pi;    
  endif
  
else %if the position isnt valid
  JointAngles = -1;
  return;
endif

end