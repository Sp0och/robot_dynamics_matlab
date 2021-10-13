function q_AC = quatMult(q_AB,q_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  Q = [q_AB(1),-q_AB(2), -q_AB(3), -q_AB(4);
       q_AB(2), q_AB(1), -q_AB(4), q_AB(3);
       q_AB(3), q_AB(4), q_AB(1), -q_AB(2);
       q_AB(4), -q_AB(3), q_AB(2), q_AB(1)];
  q_AC = Q*q_BC;
end

