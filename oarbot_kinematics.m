% initialization
clear all; close all;

%
% define unit vectors
%
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% parameters
wheel_r = 0.127;
d1 = 0.2755;
l1=0.3; l2=wheel_r+0.3+d1;
d2 = 0.41; d3=0.2073; e2=0.0098; d4=0.0741; d5=d4; d6=0.16;
aa = pi/6;
p89x = d4*(sin(aa)/sin(2*aa)) + 2*d4*(sin(aa)/sin(2*aa))*cos(2*aa);
p89y = 2*d4*sin(aa);

% initial angle
q1=0; q2=0; q3=0; q4=0; q5=0; q6=0; q7=0; q8=0; q9=0; q10=0;

% POE robot def
robot.H = [ex ey ez ez ez ey -ey -ex [-sin(pi/6); cos(pi/6); 0] -ex];
robot.P = [0*ex 0*ex 0*ex l1*ex+l2*ez 0*ex 0*ex d2*ez d3*ex+e2*ey p89x*ex-p89y*ey 0*ex d6*(sin(aa)/sin(2*aa))*ex];
robot.joint_type = [1 1 0 1 0 0 0 0 0 0];
robot.q_zeros = [0 0 0 0 pi pi pi/2 0 0 0];
robot.q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10];

robot = fwdkiniter(robot);

% test fwd and Jacobian

function robot=fwdkiniter(robot)

    J = zeros(6, length(robot.q));
    T = eye(4);
    for i=1:length(robot.H)
        if robot.joint_type(i) == 0 % revolute joint
            Ri = rot(robot.H(:,i), robot.q(i));
            pi = robot.P(:,i);
            T = T*homoT(Ri, pi);
            
            Hi = [T(1:3,1:3)*robot.H(:,i); zeros(3,1)];
        else % prismatic joint
            Ri = eye(3);
            pi = robot.P(:,i)+robot.H(:,i)*robot.q(i);
            T = T*homoT(Ri, pi);
            
            Hi = [zeros(3,1); T(1:3,1:3)*robot.H(:,i)];
        end
        
        phi = [eye(3) zeros(3,3); -hat(T(1:3,1:3)*robot.P(:,i)) eye(3)];
        J = phi*J + [zeros(6,i-1) Hi zeros(6,length(robot.H)-i)];
    end
    
    robot.T = T;
    robot.J = J;
end

function T = homoT(R, p)
    T = [R p; 0 0 0 1];
end