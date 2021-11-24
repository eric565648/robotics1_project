% initialization
clear all; close all;

%
% define unit vectors
%
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% parameters
wheel_r = 0.127;
d1 = 0.2755;
d2 = 0.41; d3=0.2073; e2=0.0098; d4=0.0741; d5=d4; d6=0.16;
aa = pi/6;
p89x = d4*(sin(aa)/sin(2*aa)) + 2*d4*(sin(aa)/sin(2*aa))*cos(2*aa);
p89y = 2*d4*sin(aa);

% initial angle
q1=0; q2=0; q3=0; q4=0; q5=0; q6=0;

% POE robot def
robot.H = [ez ey -ey -ex [-sin(pi/6); cos(pi/6); 0] -ex];
robot.P = [d1*ez 0*ez d2*ez+e2*ey 0*ex (p89x+d3)*ex-p89y*ey 0*ex (d6+d4*sin(aa)/sin(2*aa))*ex];
robot.joint_type = [0 0 0 0 0 0];
robot.joint_upper_limit = deg2rad([10000 130 90 10000 10000 10000]);
robot.joint_lower_limit = deg2rad([-10000 -130 -71 -10000 -10000 -10000]);
robot.q_zeros = [pi pi pi/2 0 0 0];

q1 = 0.1; q2=deg2rad(rand*260-130); q3=deg2rad(rand*161-71); q4=rand*2*pi-pi; q5=rand*2*pi-pi; q6=rand*2*pi-pi;
robot.q = [q1 q2 q3 q4 q5 q6];
robot = fwdkiniter(robot);

% test fwd and Jacobian

[robot_tree, col_body] = defineRobot(robot, 0.01);
figure(1);show(robot_tree,(robot.q)','collision','on');

Td = robot.T;

init_guess = [0 0 0 0 0 0];
%robot = invkiniter(robot, init_guess, 100, 1, [5 5 5 1 1 1], 0.01, 2);
robot = invkinqp(robot, init_guess, 100, 1, 0.01, 0.1, 2);
figure(2);show(robot_tree,(robot.q)','collision','on');

Tt = robot.T;

norm(Tt-Td,'fro')

function robot = invkinqp(robotd, init_guess, N, alpha, epis, Kp, type)
    
    robot_i = robotd;
    q = init_guess';

    for i=1:N
        robot_i.q = q';
        robot_i = fwdkiniter(robot_i);
        dX = [s_err(robot_i.T(1:3,1:3)*(robotd.T(1:3,1:3))',type);(robot_i.T(1:3,4)-robotd.T(1:3,4))];
        umax = (robot_i.joint_upper_limit-robot_i.q)'/alpha;
        umin = (robot_i.joint_lower_limit-robot_i.q)'/alpha;
        
        upper = [2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]';
        
        umax = (umax>upper).*(upper)+(umax<=upper).*umax;
        umin = (umin<-upper).*(-upper)+(umin>=-upper).*umin;
        
        A = [diag(ones(6,1));diag(-1*ones(6,1))];
        b = [umax; -umin];

        opt = optimset('display','off');
        %u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,[],[],[],[],umin,umax,q,opt);
        u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,A,b,[],[],[],[],q,opt);
        %u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,[],[],[],[],[],[],q,opt);
        q = q+alpha*u;
        q=(q>pi).*(-2*pi+q)+(q<-pi).*(2*pi+q)+(q<=pi).*(q>-pi).*q;
    end
    robot = robotd;
    robot.q = q';
    robot = fwdkiniter(robot);
end

function robot=invkiniter(robotd, init_guess, N, alpha, weight, epis, type)

    robot_i = robotd;
    q = init_guess';
    
    for i=1:N
        robot_i.q = q';
        robot_i = fwdkiniter(robot_i);
        
        dX = [s_err(robot_i.T(1:3,1:3)*(robotd.T(1:3,1:3))',type);(robot_i.T(1:3,4)-robotd.T(1:3,4))];
        q = q-alpha*robot_i.J'*inv(robot_i.J*robot_i.J'+epis*diag(1./weight)*eye(6))*dX;
        % q = q-alpha*pinv(robot_i.J)*dX;
        
        
        q(5:10)=(q(5:10)>=pi).*(-2*pi+q(5:10))+(q(5:10)<-pi).*(2*pi+q(5:10))+(q(5:10)<=pi).*(q(5:10)>-pi).*q(5:10);
        q(3)=(q(3)>=pi).*(-2*pi+q(3))+(q(3)<-pi).*(2*pi+q(3))+(q(3)<=pi).*(q(3)>-pi).*q(3);
    end
    
    robot = robotd;
    robot.q = q';
    robot = fwdkiniter(robot);
end

function robot=fwdkiniter(robot)

    J = zeros(6, length(robot.q));
    T = eye(4);
    for i=1:length(robot.H)
        
        phi = [eye(3) zeros(3,3); -hat(T(1:3,1:3)*robot.P(:,i)) eye(3)];
        
        if robot.joint_type(i) < 1e-5 % revolute joint
            Ri = rot(robot.H(:,i), robot.q(i));
            p = robot.P(:,i);
            T = T*homoT(Ri, p);
            
            Hi = [T(1:3,1:3)*robot.H(:,i); zeros(3,1)];
        else % prismatic joint
            Ri = eye(3);
            p = robot.P(:,i)+robot.H(:,i)*robot.q(i);
            T = T*homoT(Ri, p);
            
            Hi = [zeros(3,1); T(1:3,1:3)*robot.H(:,i)];
        end
        
        
        J = phi*J + [zeros(6,i-1) Hi zeros(6,length(robot.H)-i)];
    end
    
    % the last (fixed) joint
    phi = [eye(3) zeros(3,3); -hat(T(1:3,1:3)*robot.P(:,end)) eye(3)];
    Ri = eye(3);
    p = robot.P(:,end);
    T = T*homoT(Ri, p);
    J = phi*J;
    
    robot.T = T;
    robot.J = J;
end

function err = s_err(er_mat, type)

    qua = quaternion_from_rotation(er_mat);
    qk = angle_axis_from_rotation(er_mat);
    if type == 1
        err = 4*qua(1)*qua(2:4);
    elseif type == 2
        err = 2*qua(2:4);
    elseif type == 3
        err = 2*qk(1)*qk(2:4);
    else
        error("Type is a interger of 1, 2 or 3");
    end

end

function s=sigmafun(hI,eta,c,M,e)

    s=(hI>eta).*(-M*atan(c*(hI-eta))*2/pi)+...
        (hI>=0).*(hI<eta).*(e*(eta-hI)/eta)+(hI<0).*e;

end

function q = quaternion_from_rotation(R)
    qw = sqrt(1+R(1,1)+R(2,2)+R(3,3))/2;
    qx = (R(3,2)-R(2,3))/(4*qw);
    qy = (R(1,3)-R(3,1))/(4*qw);
    qz = (R(2,1)-R(1,2))/(4*qw);

    q = [qw; qx; qy; qz];
end

function qk = angle_axis_from_rotation(R)

    k_ = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    k = k_/norm(k_);

    sinq = k_(1)/k(1);
    cosq = (R(1,1)+R(2,2)+R(3,3)-1)/2;
    q = atan2(sinq,cosq);

    qk = [q;k];
end

function T = homoT(R, p)
    T = [R p; 0 0 0 1];
end