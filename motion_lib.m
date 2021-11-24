% initialization
clear all; close all;

oarbot_param;

kinova_param;

sr = rand*0.3 + 0.45;stheta = rand*deg2rad(130)-deg2rad(60);
sp = rand*deg2rad(90)-deg2rad(45);sy = rand*deg2rad(90)-deg2rad(45);sh = rand*0.5;
er = rand*0.3 + 0.45;etheta = rand*deg2rad(130)-deg2rad(60);
ep = rand*deg2rad(90)-deg2rad(45);ey = rand*deg2rad(90)-deg2rad(45);eh = rand*0.5;
exp_rate=0.1; exp_r = 0.1;
vel_type = 1;
arm_speed=1;base_speed = 1;

sy = 0;ey=0;

param = [sr stheta sp sy sh er etheta ep ey eh exp_rate exp_r vel_type arm_speed base_speed];
[path,pathp_d,pathq_d] = create_path(robot,armbot,sr,stheta,sp,sy,sh,er,etheta,ep,ey,eh,exp_rate,exp_r,vel_type,arm_speed,base_speed);

m = 10;
figure(1);h=plotTransforms(pathp_d(:,1:m:end)',pathq_d(:,1:m:end)','FrameSize',0.3);
view(31,12);

err = [];
for i=1:length(path(1,:))
    % show robot pose (ever 5 frames)
    robot.q = path(:,i)';
    robot = fwdkiniter(robot);
    dif = norm(robot.T-Transform(q2R(pathq_d(:,i)),pathp_d(:,i)),'fro');
    err = [err dif];
    if mod(i,5)==0
        %figure(1);show(robot_tree,path(:,i),'collision','on');
        figure(1);show(robot_tree,[0 0 0 0 0 0 0 -2.5261   -0.6155 0]','collision','on');
        hold on; h=plotTransforms(pathp_d(:,1:m:end)',pathq_d(:,1:m:end)','FrameSize',0.3); hold off;
        view(31,12);axis([0 1.5 -0.5 0.5 0 1.5]);
        rad2deg(path(:,i))
        pause(0.1);
    end
end

figure(2)
plot(err, '*');
title('Accracy of Method');
ylabel('Forbenius error'); xlabel('index'); grid;

function [path,pathp_d,pathq_d] = create_path(robot,armbot,sr,stheta,sp,sy,sh,er,etheta,ep,ey,eh,exp_rate,exp_r,vel_type,arm_speed,base_speed)

    N = 200; % TODO should related to velocity

    spoint = [sr*cos(stheta); 0; sr*sin(stheta)]+armbot.P(:,1);
    epoint = [er*cos(etheta); 0; er*sin(etheta)]+armbot.P(:,1);
    armbot.q = armbot.home; armbot = fwdkiniter(armbot);
    sspoint = armbot.T(1:3,4); ssrpy = R2rpy(armbot.T(1:3,1:3));
    
    %ebx = base_speed*self.motion_T
    ebx = 0;
    eh=0.01;sh = 0.01;

    path = zeros(length(robot.H),N+1);
    pathp_d = [];
    pathq_d = [];
    
    

%     for i=1:N
%         arm_p = (spoint-sspoint)*i/N+sspoint;
%         %TODO:Add explore
%         arm_rpy = (srpy-ssrpy)*i/N+ssrpy;
%         arm_T = Transform(rpy2R(arm_rpy),arm_p);
%         pathp_d = [pathp_d arm_T(1:3,4)+robot.P(:,4)-0.2755*[0;0;1]];pathq_d = [pathq_d R2q(arm_T(1:3,1:3))];
%         
%         armbot.T = arm_T;
%         if i==1
%             armbot = invkinqp(armbot,path(5:10,end)',2,1,0.01,1,2);
%         else
%             armbot = invkinqp(armbot,path(5:10,i-1)',2,1,0.01,1,2);
%         end
%         q = [0 0 0 0 armbot.q];
%         path(:,i) = q;
%     end
    
    arm_p = spoint;
    %TODO:Add explore
    arm_rpy = srpy;
    arm_T = Transform(rpy2R(arm_rpy),arm_p);
    pathp_d = [pathp_d arm_T(1:3,4)+robot.P(:,4)-0.2755*[0;0;1]+0.01];pathq_d = [pathq_d R2q(arm_T(1:3,1:3))];
    armbot.T = arm_T;
    %armbot = invkinqp(armbot,armbot.home,100,1,0.01,0.1,2);
    armbot = invkiniter(armbot, armbot.home, 300, 1, [5 5 5 1 1 1], 0.01, 2);
    q = [0 0 0 0 armbot.q];
    path(:,1) = q;
    
    for i=1:N
        arm_p = (epoint-spoint)*i/N+spoint;
        %TODO:Add explore
        arm_rpy = (erpy-srpy)*i/N+srpy;
        arm_T = Transform(rpy2R(arm_rpy),arm_p);
        pathp_d = [pathp_d arm_T(1:3,4)+robot.P(:,4)-0.2755*[0;0;1]+0.01];pathq_d = [pathq_d R2q(arm_T(1:3,1:3))];
        
        armbot.T = arm_T;
        %armbot = invkinqp(armbot,path(5:10,i)',70,1,0.01,0.1,2);
        armbot = invkiniter(armbot, path(5:10,i)', 300, 1, [5 5 5 1 1 1], 0.01, 2);
        sup_h = (eh-sh)/N*i+sh;
        bx = ebx/N*i;
        q = [bx 0 0 sup_h armbot.q];
        path(:,i+1) = q;
    end

end

function q=invAly(robot,pT,pitch,last)

    q = [];

    d = norm(pT-robot.P(:,1));
    p2 = robot.P(:,3);
    p1 = robot.P4T;
    k = robot.H(:,3);
    q3 = subprob3(d,p1,p2,k);

    for i=1:length(q3)
        if q3(i)>robot.joint_upper_limit(3) or q3(i)<robot.joint_upper_limit(3)
            continue;
        end
        R23 = rot(robot.H(:,3),q3(i));
        p1 = pT-robot.P(:,1);
        p2 = robot.P(:,3)+R23*robot.P4T;
        k1 = -robot.H(:,1); k2=robot.H(:.2);
        [q1,q2] = subprob2(p1,p2,k1,k2);

        for j=1:length(q1)
            if q2(j)>robot.joint_upper_limit(2) or q2(j)<robot.joint_upper_limit(2)
                continue;
            end
            R01=rot(robot.H(:,1),q1(j)); R12=rot(robot.H(:,2),q2(j));

            p1 = inv(R01*R12*R23)*[cos(pitch),0,sin(pitch)];
            p2 = ex;
            k1 = -robot.H(:,4);k2=robot.H(:,5);
            [q4,q5] = subprob2(p1,p2,k1,k2);

            for l=1:length(q4)
                if q4(l)>pi/2 or q4(l)<-pi/2
                    continue;
                end
                R34=rot(robot.H(:,4),q4(l));R45=rot(robot.H(:,5),q5(l))
                p1=ez; p2=inv(R01*R12*R23*R34*R45)*ey;
                k = robot.H(:,6);
                q6 = subprob1(p1,p2,k);
                q = [q [q1(j);q2(j);q3(i);q4(l);q5(l);q6]];
            end
        end

    end

end

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
        %upper = [0.3,0.3,0.3,0.3,0.3,0.3]';
        
        umax = (umax>upper).*(upper)+(umax<=upper).*umax;
        umin = (umin<-upper).*(-upper)+(umin>=-upper).*umin;
        
        A = [diag(ones(6,1));diag(-1*ones(6,1))];
        b = [umax; -umin];

        opt = optimset('display','off');
        %u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,[],[],[],[],umin,umax,q,opt);
        %u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,A,b,[],[],[],[],q,opt);
        u = quadprog(robot_i.J'*robot_i.J,Kp*robot_i.J'*dX,[],[],[],[],[],[],q,opt);
        q = q+alpha*u;
        q=(q>pi).*(-2*pi+q)+(q<=-pi).*(2*pi+q)+(q<=pi).*(q>-pi).*q;
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
        
        q=(q>pi).*(-2*pi+q)+(q<=-pi).*(2*pi+q)+(q<=pi).*(q>-pi).*q;
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
            T = T*Transform(Ri, p);
            
            Hi = [T(1:3,1:3)*robot.H(:,i); zeros(3,1)];
        else % prismatic joint
            Ri = eye(3);
            p = robot.P(:,i)+robot.H(:,i)*robot.q(i);
            T = T*Transform(Ri, p);
            
            Hi = [zeros(3,1); T(1:3,1:3)*robot.H(:,i)];
        end
        
        
        J = phi*J + [zeros(6,i-1) Hi zeros(6,length(robot.H)-i)];
    end
    
    % the last (fixed) joint
    phi = [eye(3) zeros(3,3); -hat(T(1:3,1:3)*robot.P(:,end)) eye(3)];
    Ri = eye(3);
    p = robot.P(:,end);
    T = T*Transform(Ri, p);
    J = phi*J;
    
    robot.T = T;
    robot.J = J;
end

function err = s_err(er_mat, type)

    qua = R2q(er_mat);
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

function q = R2q(R)
    qw = sqrt(1+R(1,1)+R(2,2)+R(3,3))/2;
    qx = (R(3,2)-R(2,3))/(4*qw);
    qy = (R(1,3)-R(3,1))/(4*qw);
    qz = (R(2,1)-R(1,2))/(4*qw);

    q = [qw; qx; qy; qz];
end

function R = q2R(q)
    % Q2R
    %   R = q2R(q)
    %   
    %   converts a quaternion into a 3 x 3 Rotation Matrix according to the
    %   Euler-Rodrigues formula.  Expects quaternion as q = [q0;qv]
    %
    %   R = I + 2*q0*hat(qv) + 2*hat(qv)^2
    R = eye(3)+2*q(1)*hat(q(2:4))+2*hat(q(2:4))*hat(q(2:4));
end

function qk = angle_axis_from_rotation(R)

    k_ = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    k = k_/norm(k_);

    sinq = k_(1)/k(1);
    cosq = (R(1,1)+R(2,2)+R(3,3)-1)/2;
    q = atan2(sinq,cosq);

    qk = [q;k];
end

function R = rpy2R(rpy)
    R = rot([0;0;1],rpy(3))*rot([0;1;0],rpy(2))*rot([1;0;0],rpy(1));
end

function rpy=R2rpy(R)
    if norm(R(1:3,1)) <= 0
        disp('Singular!!!!');
        rpy = [];
    end
    r = atan2(R(3,2),R(3,3));
    y = atan2(R(2,1),R(1,1));
    p = atan2(-R(3,1),norm(R(3,2:3)));
    rpy = [r;p;y];
end

function R=rot(k,theta)
    k=k/norm(k);
    R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
end

function T = Transform(R, p)
    T = [R p; 0 0 0 1];
end

function km=hat(k)
    km = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end

function q=subprob0(k,p1,p2)

    if ((k'*p1)>sqrt(eps) | (k'*p2)>sqrt(eps))
      error('k must be perpendicular to p and q');
    end
    
    p1=p1/norm(p1);
    p2=p2/norm(p2);
    
    q=2*atan2(norm(p1-p2),norm(p1+p2));
    
    if k'*(cross(p1,p2))<0
      q=-q;
    end
    
    q = inpi(q);
    
end

function q=subprob1(p1,p2,k)

    p1 = p1/norm(p1);
    p2 = p2/norm(p2);

    p1_p = p1 - dot(p1,k)*k;
    p2_p = p2 - dot(p2,k)*k;

    q = subprob0(k,p1_p,p2_p);

    q = inpi(q);
end

function [q1, q2]=subprob2(p1,p2,k1,k2)

    p1 = p1/norm(p1);
    p2 = p2/norm(p2);
    %p2=p2/norm(p2)*norm(p1);
    k1 = k1/norm(k1);
    k2 = k2/norm(k2);
    kk = k1.'*k2;
    
    dummy = [1 -kk; -kk 1]*[k1.'*p1; k2.'*p2]/(1-(kk)^2);
    a = dummy(1);
    b = dummy(2);

    c2 = (p1.'*p1-a^2-b^2-2*a*b*kk)/((norm(cross(k1,k2)))^2);

    if c2<0
        q1 = nan;
        q2 = nan;
        return
    elseif c2 == 0
        v1 = a*k1 + b*k2;
        v2 = nan;
    else
        v1 = a*k1 + b*k2 + sqrt(c2)*cross(k1,k2);
        v2 = a*k1 + b*k2 - sqrt(c2)*cross(k1,k2);
    end
    
    q11 = subprob1(p1,v1,k1);
    q21 = subprob1(p2,v1,k2);

    if isnan(v2)
        q12 = nan;
        q22 = nan;
    else
        q12 = subprob1(p1,v2,k1);
        q22 = subprob1(p2,v2,k2);
    end

    q11 = inpi(q11);
    q12 = inpi(q12);
    q21 = inpi(q21);
    q22 = inpi(q22);

    q1 = [q11 q12];
    q2 = [q21 q22];

end

function q=subprob3a(d,p1,p2,k)

    if norm(p2) == 0 && norm(p1) == d
        q = [0 0];
        return
    end

    cosphi = (norm(p1)^2+norm(p2)^2-d^2)/(2*norm(p1)*norm(p2));
    qq = subprob1(p1, p2, k);

    if abs(cosphi) > 1
        q = nan;
        return
    elseif abs(cosphi) == 1
        q1 = qq + acos(cosphi);
        q2 = nan;
    else
        q1 = qq + acos(cosphi);
        q2 = qq - acos(cosphi);
    end

    q1 = inpi(q1);
    q2 = inpi(q2);

    q = [q1 q2];
    
end

function q=subprob3(d, p1, p2, k)

    d_p = sqrt(d^2 - (k.'*(p1-p2))^2);
    p1_p = p1 - dot(p1,k)*k;
    p2_p = p2 - dot(p2,k)*k;

    q = subprob3a(d_p, p1_p, p2_p, k);

end