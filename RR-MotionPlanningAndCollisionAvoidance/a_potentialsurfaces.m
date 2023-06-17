%links and obstacle positions
base_x=20; base_y=0; L1=46.1; L2=26.9;
endEffector_x = 20; endEffector_y = 70;
goal_x =60; goal_y=40;
obstacle_x=40;
obstacle_y=60;
obstacle_r=10;
alpha = ((0:1:180)*pi)/180; %links' angles
beta = ((0:2:360)*pi)/180;

configurationSpace = zeros(length(alpha), length(beta));
for i = 1:length(alpha)
    for j = 1:length(beta)
        configurationSpace(i,j) = collision_check(alpha(i), beta(j), base_x, base_y, L1, L2, obstacle_x,obstacle_y,obstacle_r);
    end
end


attractiveEnergy = zeros(length(alpha), length(beta));
repulsiveEnergy = zeros(length(alpha), length(beta));
attractiveForce_a = zeros(length(alpha), length(beta));
attractiveForce_b = zeros(length(alpha), length(beta));
repulsiveForce_a = zeros(length(alpha), length(beta));
repulsiveForce_b = zeros(length(alpha), length(beta));

%calculation 
[goal_alpha, goal_beta] = inverse_kinematics(goal_x, goal_y, base_x, base_y, L1, L2);

attractiveConstant = 200;
repulsiveConstant = 5;
rho0 = 1;
for alpha1 = 1:length(alpha)
    for beta1 = 1:length(beta)
        a = alpha(alpha1);
        b = beta(beta1);
        attractiveEnergy(alpha1, beta1) = (1/2)*attractiveConstant*((a-goal_alpha).^2+(b-goal_beta).^2); %attractive energy between the current configurations
        attractiveForce_a(alpha1, beta1) = -attractiveConstant*(a-goal_alpha); %attractive forces toward to goal
        attractiveForce_b(alpha1, beta1) = -attractiveConstant*(b-goal_beta);
        rho = 2;

        if alpha1 <= 40 %[]
            a1 = 1:(alpha1+40);
        elseif alpha1 >= length(alpha)-40
            a1 = (alpha1-40):(alpha1+(length(alpha)-alpha1-1));
        else
            a1 = alpha1-40:alpha1+40;
        end

        if beta1 <= 40
            b1 = 1:(beta1+40);
        elseif beta1 >= length(beta)-40
            b1 = (beta1-40):(beta1+((length(beta)-beta1)-1));
        else
            b1 = beta1-40:beta1+40;
        end

        for i = a1
            for j = b1
                if(configurationSpace(i,j) == 1)
                    c = alpha(i);
                    d = beta(j);
                    distance = sqrt((a-c+0.1).^2+(b-d+0.1)^2); % Euclidean distance 
                else
                    distance = rho;
                end
                if(distance < rho)
                    rho = distance;
                    aRho = (a-c+0.1)/distance;
                    bRho = (b-d+0.1)/distance;
                end
            end
        end

        if rho <= rho0
            repulsiveEnergy(alpha1, beta1) = (1/2)*repulsiveConstant*(1/rho - 1/rho0)^2;
            repulsiveForce_a(alpha1, beta1) = repulsiveConstant*(1/rho - 1/rho0)*((1/rho)^2)*aRho;
            repulsiveForce_b(alpha1, beta1) = repulsiveConstant*(1/rho - 1/rho0)*((1/rho)^2)*bRho;
            if(repulsiveEnergy(alpha1, beta1) > 500)
                repulsiveEnergy(alpha1, beta1) = 500;
            end
        else
            repulsiveEnergy(alpha1, beta1) = 0;
            repulsiveForce_a(alpha1, beta1) = 0;
            repulsiveForce_b(alpha1, beta1) = 0;
        end
    end
end

%display
totalEnery = attractiveEnergy + repulsiveEnergy;
[alpha_grid, beta_grid] = meshgrid(alpha*180/pi, beta*180/pi);

figure;
subplot(2,2,1);
meshc(alpha_grid, beta_grid, totalEnery);
title('Total Potential Field');
subplot(2,2,2);
meshc(alpha_grid, beta_grid, attractiveEnergy);
title('Attractive Potential Field');
subplot(2,2,3);
meshc(alpha_grid, beta_grid, repulsiveEnergy);
title('Repulsive Potential Field');
drawnow;