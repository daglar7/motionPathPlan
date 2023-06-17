function collision = collision_check(alpha, beta, base_x, base_y, L1, L2, obstacle_x, obstacle_y, obstacle_r)

%Intersection of a Line and a Sphere 
x1 = base_x + L1*cos(alpha); %elbow positon
y1 = base_y + L1*sin(alpha);
x2 = x1 + L2*cos(alpha+beta); %end effector position
y2 = y1+L2*sin(alpha+beta);
x3 = obstacle_x;
y3 = obstacle_y;

a = (x2 - x1)^2 + (y2 - y1)^2;
b = 2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
c = x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-obstacle_r^2;
u = ((x3-x1)*(x2-x1)+(y3-y1)*(y2-y1))/((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
delta = b*b - 4*a*c;

xaxes = all([0, x2, x1, 100] >= 0) && all([0, x2, x1, 100] <= 100); %workspace border [0,100,0,100]
yaxes = all([0, y2, y1, 100] >= 0) && all([0, y2, y1, 100] <= 100);

alphaDegree = any(alpha <= 0 | alpha >= pi); %[0,180]
betaDegree = any(beta <= 0 | beta >= 2*pi); %[0,360]



if (0 <= u && u <= 1 && delta >= 0) || (~xaxes || ~yaxes || alphaDegree || betaDegree )    
    collision = 1;
else
    collision = 0;
end
end
