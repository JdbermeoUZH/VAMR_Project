function [ T_21 ] = invt( T_12 )
% Returns the inverse homogeneous transformation
% Input:
%  - T_12(4x4) : homogeneous transformation from Frame 2 to 1

% Output:
%  - T_21(4x4) : homogeneous transformation from Frame 1 to 2

T_21 = [T_12(1:3,1:3)',  -T_12(1:3,1:3)'*T_12(1:3,4);
          zeros(1,3),                1];   

end