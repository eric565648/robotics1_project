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
q1=1; q2=1; q3=0; q4=0; q5=0; q6=0; q7=0; q8=0; q9=0; q10=0;
q1=0; q2=0; q3=0; q4=0; q5=0; q6=0; q7=0; q8=0; q9=0; q10=0;

% POE robot def
robot.H = [ex ey ez ez ez ey -ey -ex [-sin(pi/6); cos(pi/6); 0] -ex];
robot.P = [0*ex 0*ex 0*ex l1*ex+l2*ez 0*ex 0*ex d2*ez d3*ex+e2*ey p89x*ex-p89y*ey 0*ex (d6+d4*sin(aa)/sin(2*aa))*ex];
robot.joint_type = [1 1 0 1 0 0 0 0 0 0];
robot.joint_upper_limit = [100 100 100 0.5 deg2rad([10000 130 180 10000 10000 10000])];
robot.joint_lower_limit = [-100 -100 -100 0 deg2rad([-10000 -130 -71 -10000 -10000 -10000])];
robot.q_zeros = [0 0 0 0 pi pi pi/2 0 0 0];

robot.q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10];

robot = fwdkiniter(robot);

% test fwd and Jacobian

[robot_tree, col_body] = defineRobot(robot, 0.01);
figure(1);show(robot_tree,(robot.q)','collision','on'); hold;

% draw a convex shape as path
x = [0.8, 3.5, 6.2];
z = [1.11,1.5,1.11];
p = polyfit(x,z,2);
x_end = 7.5; z_end=p(1)*x_end*x_end + p(2)*x_end + p(3);
N = 100;
S_length = N+1;
p_Sx = x(1):(x_end-x(1))/N:x_end;
p_Sz = p(1)*p_Sx.*p_Sx + p(2)*p_Sx + p(3);
p_S = [p_Sx; p_Sz];

% make it equal length
diffS=vecnorm(diff(p_S')');
ls=[0 cumsum(diffS)];
lf=sum(diffS);
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')';
p_Ts=[pS(1,:);zeros(1,length(pS(1,:)));pS(2,:)];

R_Ts = [];
qua_Ts = [];
for i=1:S_length
    
    eyt = [0;1;0];
    
    if i == S_length
        ext = (p_Ts(:,i)-p_Ts(:,i-1));
    else
        ext = (p_Ts(:,i+1)-p_Ts(:,i));
    end
    
    ext = ext - ext'*eyt*eyt;
    ext = ext/norm(ext);
    ezt = cross(ext,eyt);
    
    R = [ext,eyt,ezt];
    q = quaternion_from_rotation(R);
    
    R_Ts = [R_Ts R];
    qua_Ts = [qua_Ts q];
end

m=3;
%figure(2);
h=plotTransforms(p_Ts(:,1:m:end)',qua_Ts(:,1:m:end)');
set(h,'LineWidth',1.5);

ik = inverseKinematics('RigidBodyTree',robot_tree);

all_sol = [];
all_error = [];
for i=1:S_length

    T = [[R_Ts(:,3*i-2:3*i) p_Ts(:,i)];[0 0 0 1]];
    robot.T = T;

    if i==1
        init_guess = [0 0 0 0 0 0 0 pi/6 pi/6 pi/6];
    else
        init_guess = all_sol(:,i-1)';
    end

    %robot = invkiniter(robot, init_guess, 100, 1, [5 5 5 1 1 1], 0.01, 2);
    robot = invkinqp(robot, init_guess, 1, 1, 0.01, 1, 2);
    
%     [q,solnInfo]=...
%        ik('body11',T,[1 1 1 1 1 1],init_guess');
    
    all_sol = [all_sol robot.q'];
%     all_sol = [all_sol q];
    error_T = norm(robot.T-T,'fro');
    all_error = [all_error error_T];
    %figure(1);view(0,10);axis([-1 10 -1 1 -1 2]);show(robot_tree,all_sol(:,i),'collision','on');
end

for i=1:N
    % show robot pose (every m frames)
    if mod(i,m)==0
        robot.q = all_sol(:,i)';
        robot = fwdkiniter(robot);
        
        figure(1);axis([-1 9 -1 1 -1 2]);show(robot_tree,all_sol(:,i),'collision','on'); 
        pause(0.1);
        view(0,10);
    end
end

figure(2)
plot(all_error, '*');
title('Accracy of Method');
ylabel('Forbenius error'); xlabel('index'); grid;

function robot = invkinqp(robotd, init_guess, N, alpha, epis, Kp, type)
    
    robot_i = robotd;
    q = init_guess';

    for i=1:N
        robot_i.q = q';
        robot_i = fwdkiniter(robot_i);
        dX = [s_err(robot_i.T(1:3,1:3)*(robotd.T(1:3,1:3))',type);(robot_i.T(1:3,4)-robotd.T(1:3,4))];
        umax = (robot_i.joint_upper_limit-robot_i.q)'/alpha;
        umin = (robot_i.joint_lower_limit-robot_i.q)'/alpha;
        
        upper = [1,1,2*pi,1,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]';
        
        umax = (umax>upper).*(upper)+(umax<=upper).*umax;
        umin = (umin<-upper).*(-upper)+(umin>=-upper).*umin;
        
        %A = [diag(ones(10,1));diag(-1*ones(10,1))];
        %b = [(robot_i.joint_upper_limit-robot_i.q)'/alpha; (-robot_i.joint_lower_limit+robot_i.q)'/alpha];

        opt = optimset('display','off');
        u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,[],[],[],[],umin,umax,q,opt);
        %u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,A,b,[],[],[],[],q,opt);
        %u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,[],[],[],[],[],[],q,opt);
        q = q+alpha*u;
        q(5:10)=(q(5:10)>=2*pi).*(-2*pi+q(5:10))+(q(5:10)<0).*(2*pi+q(5:10))+(q(5:10)<2*pi).*(q(5:10)>=0).*q(5:10);
        q(3)=(q(3)>=2*pi).*(-2*pi+q(3))+(q(3)<0).*(2*pi+q(3))+(q(3)<2*pi).*(q(3)>=0).*q(3);
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