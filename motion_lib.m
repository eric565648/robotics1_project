% initialization
 close all;
% clear all;

oarbot_param;

kinova_param;

dataN=1;
paramM=[];

plotvel = [];

i=1;
while i<=dataN

%     sr = rand*0.35 + 0.3;stheta = rand*deg2rad(150)-deg2rad(60);
%     sp = rand*deg2rad(180)-deg2rad(90);sh = rand*0.3; shand=rand;
%     er = rand*0.35 + 0.3;etheta = rand*deg2rad(150)-deg2rad(60);
%     ep = rand*deg2rad(180)-deg2rad(90);eh = rand*0.3; ehand=rand;
%     exp_r = rand-2;
%     arm_speed = rand-2;
%     vel_type=randi([1,6]);
%     base_speed=rand*2-2;
    
    sr = 0.48; stheta=deg2rad(0);
    er = 0.5; etheta=deg2rad(60);
    exp_r = -2;
    arm_speed=-1;
    base_speed=-2;
    vel_type=1;

%     disp([sr stheta sp sh])
%     disp([er etheta ep eh])
%     disp(exp_r)
%     disp([arm_speed vel_type base_speed])

    [path,pathp_d,pathq_d] = create_path(robot,armbot,sr,stheta,sp,sh,shand,er,etheta,ep,eh,ehand,exp_r,vel_type,arm_speed,base_speed);

    if ~isempty(path)
%         tic
        for k=1:length(path(1,:))
            qbot=path(1:end-1,k);
            figure(1);show(robot_tree,qbot,'collision','on','PreservePlot',0,'FastUpdate',1);
            view(43,16);axis([0 1 -0.5 0.5 0 2]);
        end
%         toc
%         disp(length(path(1,:))*0.02)
        paramM = [paramM;sr,stheta,sp,sh,shand,er,etheta,ep,eh,ehand,exp_r,vel_type,arm_speed,base_speed];
        writematrix(path,'motion_data_test/'+string(i)+'.csv');
        fprintf('Number %s\n',num2str(i));

%         figure(i);plot(vecnorm(diff(pathp_d')')/0.02);title('Velocity Type '+string(i));
%         ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
        i=i+1;
%         plotvel = [plotvel;vecnorm(diff(pathp_d')')/0.02];
    end
end
writematrix(paramM,'motion_data/data_param.csv');

% figure(1);plot(plotvel(1,:));title('Velocity Type 1');
% ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
% figure(2);plot(plotvel(2,:));title('Velocity Type 1');
% ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
% figure(3);plot(plotvel(3,:));title('Velocity Type 1');
% ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
% figure(4);plot(plotvel(4,:));title('Velocity Type 1');
% ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
% figure(5);plot(plotvel(5,:));title('Velocity Type 1');
% ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
% figure(6);plot(plotvel(6,:));title('Velocity Type 1');
% ylabel('Path Speed m/s'); xlabel('Time Index'); grid;

function [path,pathp_d,pathq_d] = create_path(robot,armbot,sr,stheta,sp,sh,shand,er,etheta,ep,eh,ehand,exp_r,vel_type,arm_speed,base_speed)

    %N = 50; % TODO should related to velocity
    s_rate = 0.02; % sampling rate

    spoint = [sr*cos(stheta); 0; sr*sin(stheta)]+armbot.P(:,1);
    epoint = [er*cos(etheta); 0; er*sin(etheta)]+armbot.P(:,1);
    
    %N = norm(epoint-spoint)/exp_r*exp_rate;
    N = norm(epoint-spoint)*50;
    
    exp_r = 10^exp_r;
    arm_speed = 10^arm_speed;
    base_speed = 10^base_speed;
    

    path_p = [spoint];
    for i=1:N
        sl=epoint-spoint;
        expp=sl*i/N+spoint;
        expp_xz = randn*(exp_r/3);
        th_xz = atan2(sl(3),sl(1))+pi/2;
        expp_y = randn*(exp_r/3);
        expp = expp + [expp_xz*cos(th_xz);expp_y;expp_xz*sin(th_xz)];
        path_p = [path_p expp];
    end
    
    path_vel_p = [spoint];
    for i=1:N
        j=1;
        while 1
            % propogate s_rate*arm_speed with in the direction
            direction = (path_p(:,i+1)-path_p(:,i))/norm(path_p(:,i+1)-path_p(:,i));
            this_p = path_p(:,i) + direction*(s_rate*arm_speed)*j;
            
            if norm(this_p-path_p(:,i)) > norm(path_p(:,i+1)-path_p(:,i))
                break;
            else
                path_vel_p = [path_vel_p this_p];
                j=j+1;
            end
        end
    end
    
    if length(path_vel_p(1,:)) <= 50
        disp('Weird short path problem')
        path=[];pathp_d=[];pathq_d=[];
        return;
    end
    path_type_p = [];
    total_T = length(path_vel_p)*s_rate;
    if total_T > 10 % at most 10 sec of a motion
        total_T = 10;
    end
    total_N = total_T/s_rate;
    acc_N = total_N/5;
    acc_N_L = (total_N-acc_N)/2+1;
    low_vel = 0.75;
    high_vel = 1+(1-low_vel);
    if vel_type == 1
        path_type_p = path_vel_p(:,1:total_N);
    elseif vel_type==2 || vel_type==3
        if vel_type==2
            vel_1=high_vel;vel_2=low_vel;
        elseif vel_type==3
            vel_1=low_vel;vel_2=high_vel;
        end
        
        path_type_p = [path_type_p path_vel_p(:,1)];
        pidx=1;
        for i=2:acc_N_L
            %pi = (i-1)*high_vel+1;
            pidx = pidx+vel_1;
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            path_type_p = [path_type_p this_p];
        end
        for i=acc_N_L+1:acc_N_L+acc_N
            pidx = pidx+(vel_1-(vel_1-vel_2)*(i-acc_N_L)/acc_N);
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            path_type_p = [path_type_p this_p];
        end
        for i=acc_N_L+1+acc_N:total_N
            pidx = pidx+vel_2;
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            if pi_i < length(path_vel_p(1,:))
                this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            else
                this_p = path_vel_p(:,pi_i);
            end
            path_type_p = [path_type_p this_p];
        end
    elseif vel_type==4 || vel_type==5 || vel_type==6
        if vel_type==4
            vel_1=1;vel_2=1;
        elseif vel_type==5
            vel_1=high_vel;vel_2=low_vel;
        elseif vel_type==6
            vel_1=low_vel;vel_2=high_vel;
        end
        
        path_type_p = [path_type_p path_vel_p(:,1)];
        pidx=1;
        for i=2:acc_N_L
            pidx = pidx+vel_1;
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            path_type_p = [path_type_p this_p];
        end
        for i=acc_N_L+1:acc_N_L+acc_N/3
            pidx = pidx+(vel_1-(vel_1)*(i-acc_N_L)/(acc_N/3));
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            path_type_p = [path_type_p this_p];
        end
        for i=acc_N_L+acc_N/3+1:acc_N_L+2*acc_N/3
            pidx = pidx;
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            path_type_p = [path_type_p this_p];
        end
        for i=acc_N_L+2*acc_N/3+1:acc_N_L+acc_N
            pidx = pidx+(vel_2*(i-acc_N_L-2*acc_N/3)/(acc_N/3));
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            path_type_p = [path_type_p this_p];
        end
        for i=acc_N_L+1+acc_N:total_N
            pidx = pidx+vel_2;
            pi_i=floor(pidx); pi_d=pidx-floor(pidx);
            if pi_i < length(path_vel_p(1,:))
                this_p = path_vel_p(:,pi_i)+(path_vel_p(:,pi_i+1)-path_vel_p(:,pi_i))*pi_d;
            else
                this_p = path_vel_p(:,pi_i);
            end
            path_type_p = [path_type_p this_p];
        end
    end
    
    path = [];
    pathp_d = [];
    pathq_d = [];
    
    %path_N = length(path_vel_p);
    path_N = length(path_type_p);
    for i=1:path_N
        %arm_p = (epoint-spoint)*i/N+spoint;
        %arm_p = path_p(:,i);
        %arm_p = path_vel_p(:,i);
        arm_p = path_type_p(:,i);
        arm_pitch = (ep-sp)*i/path_N+sp;
        arm_T = Transform(rpy2R([0;arm_pitch;0]),arm_p);
        pathp_d = [pathp_d arm_T(1:3,4)+robot.P(:,4)-0.2755*[0;0;1]+0.01];pathq_d = [pathq_d R2q(arm_T(1:3,1:3))];
        
        armbot.T = arm_T;
        %armbot = invkinqp(armbot,path(5:10,i)',70,1,0.01,0.1,2);
        % armbot = invkiniter(armbot, path(5:10,i)', 300, 1, [5 5 5 1 1 1], 0.01, 2);
        
        if i==1
            qbot = invAly(armbot,arm_p,arm_pitch,[]);
        else
            qbot = invAly(armbot,arm_p,arm_pitch,path(5:10,end));
        end
        
        sup_h = (eh-sh)/path_N*i+sh;
        sup_hand = (ehand-shand)/path_N*i+shand;
        sup_h = (eh-sh)/path_N*i+sh;
        bx = base_speed*s_rate*(i-1);
        if isempty(qbot) || sum(isnan(qbot))
%             disp('No solution')
            path=[];
            break;
        end
        if i>1
%             abs((qbot-path(5:end,end)))/s_rate
%             armbot.joint_vel_limit
            if sum(abs((qbot-path(5:10,end)))/s_rate > armbot.joint_vel_limit)
%                 disp('Joint too fast')
                path=[];
                break;
            end
        end
        
        q = [bx 0 0 sup_h qbot' sup_hand];
        path = [path q'];
        disp('');
    end

end

function qans=invAly(robot,pT,pitch,last)

    q = [];
    qans=[];

    d = norm(pT-robot.P(:,1));
    p2 = robot.P(:,3);
    p1 = -robot.P4T;
    k = robot.H(:,3);
    q3 = subprob3(d,p1,p2,k);

    for i=1:length(q3)
        if q3(i)>robot.joint_upper_limit(3) || q3(i)<robot.joint_lower_limit(3)
            continue;
        end
        R23 = rot(robot.H(:,3),q3(i));
        p1 = pT-robot.P(:,1);
        p2 = robot.P(:,3)+R23*robot.P4T;
        k1 = -robot.H(:,1); k2=robot.H(:,2);
        [q1,q2] = subprob2(p1,p2,k1,k2);

        for j=1:length(q1)
            if q2(j)>robot.joint_upper_limit(2) || q2(j)<robot.joint_lower_limit(2) || q1(j)>robot.joint_upper_limit(1) || q1(j)<robot.joint_lower_limit(1)
                continue;
            end
            R01=rot(robot.H(:,1),q1(j)); R12=rot(robot.H(:,2),q2(j));

            p1 = inv(R01*R12*R23)*[cos(-pitch);0;sin(-pitch)];
            p2 = [1;0;0];
            k1 = -robot.H(:,4);k2=robot.H(:,5);
            [q4,q5] = subprob2(p1,p2,k1,k2);

            for l=1:length(q4)
%                 if q4(l)>pi/2 || q4(l)<-pi/2
%                     continue;
%                 end
                R34=rot(robot.H(:,4),q4(l));R45=rot(robot.H(:,5),q5(l));
                p1=[0;0;1]; p2=inv(R01*R12*R23*R34*R45)*[0;1;0];
                k = robot.H(:,6);
                q6 = subprob1(p1,p2,k);
                q = [q [q1(j);q2(j);q3(i);q4(l);q5(l);q6]];
            end
        end

    end
    
%     disp(length(q(1,:)))
%     if length(q(1,:))>1
%         disp("more than one ans");
%     end

    if isempty(q) ~= 1
        if isempty(last) ~= 1
            qi = 1;
            sm=norm(q(:,qi)-last);
            for i=1:length(q(1,:))
                diffq = q(:,i)-last;
                for j=1:length(diffq)
                    diffq(j)=inpi(diffq(j));
                end
                if norm(diffq)<sm
                    sm=norm(diffq);
                    qi=i;
                end
            end
            qans = q(:,qi);
        else
            for i=1:length(q(1,:))
                if q(4,i)<=pi/2 && q(4,i)>=-pi/2
                    qans = q(:,i);
                    break;
                end
            end
        end
    end
    
    for i=1:length(qans)
        qans(i)=inpi(qans(i));
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

function q_inpi=inpi(q)
    while q > pi
        q = q-2*pi;
    end
    while q < -pi
        q = q+2*pi;
    end
    q_inpi = q;
end