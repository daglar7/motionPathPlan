function [alpha, beta] = inverse_kinematics(endEffector_x, endEffector_y, base_x, base_y, L1, L2)

cos_beta = ((endEffector_x-base_x)^2+(endEffector_y-base_y)^2-L1^2-L2^2)/(2*L1*L2);
sin_beta = sqrt(1-cos_beta^2);

beta = atan2(sin_beta, cos_beta);
if(beta < 0)
    beta = beta + 2*pi;
elseif(beta > 2*pi)
    beta = beta - 2*pi;
end

alpha = atan2((endEffector_y-base_y), (endEffector_x-base_x))-atan2((L2*sin(beta)),L1+L2*cos(beta)); 

end

