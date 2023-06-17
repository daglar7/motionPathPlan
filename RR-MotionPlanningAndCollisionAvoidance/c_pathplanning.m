clear
clc  
close all
%% 
%two links and obstacle

figure(1);

xborder=[0,100];
yborder=[0,100];
line([xborder(1),xborder(2),xborder(2),xborder(1)],[yborder(1),yborder(1),yborder(2),yborder(2)]);
hold on;
grid on;

base_x=20; base_y=0; L1=46.1; L2=26.9;
endEffector_x = 20; endEffector_y = 70; %start position
goal_x = 60; goal_y= 40; % goal position

%obstacle
obstacle_x=40; obstacle_y=60; obstacle_r=10;
theta = linspace(0, 2*pi, 73);
x_circle = obstacle_x + obstacle_r*cos(theta);
y_circle = obstacle_y + obstacle_r*sin(theta);
line([x_circle, x_circle(1)], [y_circle, y_circle(1)]);
plot(endEffector_x,endEffector_y,'x', 'MarkerFaceColor', 'r')
plot(goal_x,goal_y,'x', 'MarkerFaceColor', 'r')

%plot links
[start_alpha, start_beta] = inverse_kinematics(endEffector_x, endEffector_y, base_x, base_y, L1, L2);
x1 = base_x + L1*cos(start_alpha); %elbow positon
y1 = base_y + L1*sin(start_alpha);
x2 = x1 + L2*cos(start_alpha+start_beta); %end effector position
y2 = y1+L2*sin(start_alpha+start_beta);

L1_x = [base_x, x1];
L1_y = [base_y, y1];
L2_x = [x1, x2];
L2_y = [y1, y2];

L1_link = line(L1_x, L1_y);
set(L1_link, 'Color', 'b'); 

L2_link = line(L2_x, L2_y);
set(L2_link, 'Color', 'g'); 


L3_link = line(L2_x, L2_y);
set(L3_link, 'Color', 'g');

L4_link = plot(L2_x(2), L2_y(2), 'ok', 'MarkerSize', 8, 'MarkerFaceColor', 'k');



%%
%configuration space

[goal_alpha, goal_beta] = inverse_kinematics(goal_x, goal_y, base_x, base_y, L1, L2);

alpha = ((0:1:180)*pi)/180; %links' angles
beta = ((0:2:360)*pi)/180;
% convert to configuration space
configurationSpace = zeros(length(alpha), length(beta));
for i = 1:length(alpha)
    for j = 1:length(beta)
        configurationSpace(i,j) = collision_check(alpha(i), beta(j), base_x, base_y, L1, L2, obstacle_x,obstacle_y,obstacle_r);
    end
end

[alpha_grid, beta_grid] = meshgrid(alpha*180/pi, beta*180/pi);

figure(2);
start_alpha_c = find(abs(alpha-start_alpha) == min(abs(alpha-start_alpha)));
start_beta_c = find(abs(beta - start_beta) == min(abs(beta - start_beta)));
alpha_goal_c = find(abs(alpha - goal_alpha) == min(abs(alpha - goal_alpha)));
beta_goal_c = find(abs(beta - goal_beta) == min(abs(beta - goal_beta)));
configurationSpace(start_alpha_c, start_beta_c) = 1.5;
cs_figure = meshc(alpha_grid, beta_grid, configurationSpace);
view(2);
%% Find total forces
% same calculations from a_potentialsurfaces
attractiveConstant = 500;
repulsiveConstant = 5;
rho0 = 1;

attractiveEnergy = zeros(length(alpha), length(beta));
repulsiveEnergy = zeros(length(alpha), length(beta));
attractiveForce_a = zeros(length(alpha), length(beta));
attractiveForce_b = zeros(length(alpha), length(beta));
repulsiveForce_a = zeros(length(alpha), length(beta));
repulsiveForce_b = zeros(length(alpha), length(beta));

% Calculate potentials and forces for the whole grid
for alpha1 = 1:length(alpha)
    for beta1 = 1:length(beta)
        a = alpha(alpha1);
        b = beta(beta1);
        attractiveEnergy(alpha1, beta1) = (1/2)*attractiveConstant*((a-goal_alpha).^2+(b-goal_beta).^2);
        attractiveForce_a(alpha1, beta1) = -attractiveConstant*(a-goal_alpha);
        attractiveForce_b(alpha1, beta1) = -attractiveConstant*(b-goal_beta);
        rho = 2;

        if alpha1 <= 40
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
                    dist = sqrt((a-c+0.1).^2+(b-d+0.1)^2);
                else
                    dist = rho;
                end
                if(dist < rho)
                    rho = dist;
                    aRho = (a-c+0.1)/dist;
                    bRho = (b-d+0.1)/dist;
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

totalForce_a = attractiveForce_a + repulsiveForce_a;
totalForce_b = attractiveForce_b + repulsiveForce_b;
%%
%path planning

delta = 0.01;
wSteps = 20;
minDistance = 20;
start_point = [start_alpha, start_beta];
path = start_point;


x1 = base_x + L1*cos(start_alpha); %elbow positon
y1 = base_y + L1*sin(start_alpha);
x2 = x1 + L2*cos(start_alpha+start_beta); %end effector position
y2 = y1+L2*sin(start_alpha+start_beta);

i = 1;
while (((x2 - goal_x)^2 + (y2 - goal_y)^2) > 1)  % Euclidean distance 
    a2 = find(abs(alpha-path(i,1)) == min(abs(alpha-path(i,1))));
    b2 = find(abs(beta-path(i,2)) == min(abs(beta-path(i,2))));
    forceNormal = sqrt(totalForce_a(a2, b2)^2+totalForce_b(a2,b2)^2);
    forceGradient = [totalForce_a(a2,b2)/forceNormal, totalForce_b(a2,b2)/forceNormal];
    
     if(i>minDistance)
        if(sqrt(sum((path(i,:)-path(i-minDistance,:)).^2))<delta) % Euclidean distance between two points
            wPath = WaveFront(configurationSpace, alpha, beta, a2, b2, alpha_goal_c, beta_goal_c, wSteps);
            path(i+1:i+length(wPath),:) = [alpha(wPath(1,:))', beta(wPath(2,:))'];
            for j = 1:length(wPath)
                a = path(i+j,1);
                b = path(i+j,2);
              
                x1 = base_x + L1*cos(a); %elbow positon
                y1 = base_y + L1*sin(a);
                x2 = x1 + L2*cos(a+b); %end effector position
                y2 = y1+L2*sin(a+b);
                L1_x = [base_x, x1];
                L1_y = [base_y, y1];
                L2_x = [x1, x2];
                L2_y = [y1, y2];    

                set(L1_link, 'XData', L1_x, 'YData', L1_y);
                set(L2_link, 'XData', L1_x(2), 'YData', L1_y(2));
                set(L3_link, 'XData', L2_x, 'YData', L2_y);
                set(L4_link, 'XData', L2_x(2), 'YData', L2_y(2));

                configurationSpace(a2,b2) = 1.5;
                set(cs_figure, 'ZData', configurationSpace);

                drawnow;    
            end
            i = i+length(wPath);
        else
            path(i+1,:) = path(i,:) + delta*forceGradient;
            a = path(i,1);
            b = path(i,2);
            
            x1 = base_x + L1*cos(a); %elbow positon
            y1 = base_y + L1*sin(a);
            x2 = x1 + L2*cos(a+b); %end effector position
            y2 = y1+L2*sin(a+b);
            L1_x = [base_x, x1];
            L1_y = [base_y, y1];
            L2_x = [x1, x2];
            L2_y = [y1, y2];    
            i = i + 1;
        
            set(L1_link, 'XData', L1_x, 'YData', L1_y);
            set(L2_link, 'XData', L1_x(2), 'YData', L1_y(2));
            set(L3_link, 'XData', L2_x, 'YData', L2_y);
            set(L4_link, 'XData', L2_x(2), 'YData', L2_y(2));

            configurationSpace(a2,b2) = 1.5;
            set(cs_figure, 'ZData', configurationSpace);

            drawnow;
        end
    else
        path(i+1,:) = path(i,:) + delta*forceGradient;
        a = path(i,1);
        b = path(i,2);
       
        x1 = base_x + L1*cos(a); %elbow positon
        y1 = base_y + L1*sin(a);
        x2 = x1 + L2*cos(a+b); %end effector position
        y2 = y1+L2*sin(a+b);
        L1_x = [base_x, x1];
        L1_y = [base_y, y1];
        L2_x = [x1, x2];
        L2_y = [y1, y2];    
        i = i + 1;
        
        set(L1_link, 'XData', L1_x, 'YData', L1_y);
        set(L2_link, 'XData', L1_x(2), 'YData', L1_y(2));
        set(L3_link, 'XData', L2_x, 'YData', L2_y);
        set(L4_link, 'XData', L2_x(2), 'YData', L2_y(2));

        configurationSpace(a2,b2) = 1.5;
        set(cs_figure, 'ZData', configurationSpace);

        drawnow;
    end
end