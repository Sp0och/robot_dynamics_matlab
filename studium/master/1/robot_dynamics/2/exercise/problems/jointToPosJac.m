function J_P = jointToPosJac(q)
    % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector orientation which maps joint
  % velocities to end-effector angular velocities in I frame.

  % Compute the rotational jacobian.
  
  
  T12 = jointToTransform12(q);
  T23 = jointToTransform23(q);
  T34 = jointToTransform34(q);
  T45 = jointToTransform45(q);
  T56 = jointToTransform56(q);
  
  TI1 = jointToTransform01(q);
  TI2 = TI1*T12;
  TI3 = TI2*T23;
  TI4 = TI3*T34;
  TI5 = TI4*T45;
  TI6 = TI5*T56;
  
  CI1 = TI1(1:3,1:3);
  CI2 = TI2(1:3,1:3);
  CI3 = TI3(1:3,1:3);
  CI4 = TI4(1:3,1:3);
  CI5 = TI5(1:3,1:3);
  CI6 = TI6(1:3,1:3);
  
  r1 = TI1(1:3,4);
  r2 = TI2(1:3,4);
  r3 = TI3(1:3,4);
  r4 = TI4(1:3,4);
  r5 = TI5(1:3,4);
  r6 = TI6(1:3,4);
  
  rIE = jointToPosition(q);
  
  n1 = [0;0;1];
  n2 = [0;1;0];
  n3 = [0;1;0];
  n4 = [1;0;0];
  n5 = [0;1;0];
  n6 = [1;0;0];
  
  nI1 = CI1*n1;
  nI2 = CI2*n2;
  nI3 = CI3*n3;
  nI4 = CI4*n4;
  nI5 = CI5*n5;
  nI6 = CI6*n6;
  
  
  J_P = [cross(nI1,rIE-r1), cross(nI2,rIE-r2), cross(nI3,rIE-r3), cross(nI4,rIE-r4), cross(nI5,rIE-r5), cross(nI6,rIE-r6)];
    
end