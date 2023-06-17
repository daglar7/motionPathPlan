figure;
xborder=[0,100];
yborder=[0,100];
line([xborder(1),xborder(2),xborder(2),xborder(1)],[yborder(1),yborder(1),yborder(2),yborder(2)]);
hold on;
grid on;

%2 Links Robot
base_x=20; base_y=0; L1=46.1; L2=26.9;
endEffector_x = 20; endEffector_y = 70; %start position
goal_x = 60; goal_y= 40; % goal position

[start_alpha, start_beta] = inverse_kinematics(endEffector_x, endEffector_y, base_x, base_y, L1, L2);

x1 = base_x + L1*cos(start_alpha); %elbow positon
y1 = base_y + L1*sin(start_alpha);
x2 = x1 + L2*cos(start_alpha+start_beta); %end effector position
y2 = y1+L2*sin(start_alpha+start_beta);


L1start_x = [base_x, x1];
L1start_y = [base_y, y1];
L2start_x = [x1, x2];
L2start_y = [y1, y2];

%obstacle 
obstacle_x=40; obstacle_y=60; obstacle_r=10;
theta = linspace(0, 2*pi, 73);
x_circle = obstacle_x + obstacle_r*cos(theta);
y_circle = obstacle_y + obstacle_r*sin(theta);
line([x_circle, x_circle(1)], [y_circle, y_circle(1)]);

drawnow;

% plot initial and goal location of the robot
plot(endEffector_x, endEffector_y, 'x', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
plot(goal_x, goal_y, 'x', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); 

L1_link = line(L1start_x, L1start_y);
set(L1_link, 'Color', 'b'); 

L2_link = line(L2start_x, L2start_y);
set(L2_link, 'Color', 'g'); 
