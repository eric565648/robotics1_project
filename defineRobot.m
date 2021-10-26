%
% defineRobot.m
% Define a robot as a collision body rigid body tree given the
% Product of Expoential (POE) description of a robot arm
%
% input
%       robdef: robot definition structure with
%               robdef.H: motion axis, a 3xn matrix
%               robdef.P: link vector, a 3x(n+1) matrix
%               robdef.joint_type: 1xn vector, 0 for revolute 1 for prismatic
%       rad: radius of the link cylinder (each link assumed to be a
%       cylinder of equal radius).  This is easily changed in the
%       code to have different radii or link shape (e.g., box)
%
% output      
%       robot: MATLAB rigid body tree object
%       colLink: MATLAB link collision body description (in zero
%       configuration) 
%
function [robot,colLink]=defineRobot(robdef,rad)

% product of exponential description
H=robdef.H;
P=robdef.P;
type=robdef.joint_type;
n=numel(type);

% homogeneous transforms T0i stored in T0i{k}, k=1,..,n+1
T=eye(4,4);
for i=1:n
    q(i)=0;
    if type(i)==0
        Ti{i}=[rot(H(:,i),q(i)) P(:,i);[0 0 0 1]];
        T=T*Ti{i};
    else
        Ti{i}=[eye(3,3),P(:,i)+q(i)*H(:,i);[0 0 0 1]];
        T=T*Ti{i};
    end
    T0i{i}=T;
end
Ti{n+1}=[eye(3,3) P(:,n+1);[0 0 0 1]];
T0i{n+1}=T*Ti{n+1};

% define MATLAB rigidbody tree

robot=rigidBodyTree('DataFormat','column');

for i=1:n
    ii=num2str(i);
    eval(['body',ii,' = rigidBody(''','body',ii,''');']);
    if type(i)==0
        eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
            ii,'''',',''revolute''',');']);
    else
        eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
            ii,'''',',''prismatic''',');']);
    end
    eval(['jnt',ii,'.JointAxis=H(:,',ii,');']);
end
ii=num2str(n+1);
eval(['body',ii,' = rigidBody(''','body',ii,''');']);
eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
    ii,'''',',''fixed''',');']);

for i=1:n+1
    ii=num2str(i);
    eval(['setFixedTransform(jnt',ii,',Ti{',ii,'});']);
    eval(['body',ii,'.Joint = jnt',ii,';']);
end

addBody(robot,body1,'base')
for i=2:n+1
    ii=num2str(i);
    iim1=num2str(i-1);
    eval(['addBody(robot,body',ii,',','''body',iim1,''');']);
end

colBodyRadius=rad;

for i=1:n+1
    if norm(P(:,i))>0
        if (i<n+1) && (type(i)>0)
            colLink{i} = collisionBox(colBodyRadius,colBodyRadius,norm(P(:,i)));
        else
            colLink{i} = collisionCylinder(colBodyRadius,norm(P(:,i)));
        end
        kvec=cross([0;0;1],P(:,i));
        if norm(kvec)<sqrt(eps)
            colLink{i}.Pose=trvec2tform(P(:,i)'/2);
        else
            th=subprob0(kvec,[0;0;1],P(:,i));
            colLink{i}.Pose=[rot(kvec,th) P(:,i)/2;[0 0 0 1]];
        end
    else
        colLink{i} = collisionCylinder(colBodyRadius,norm(P(:,i)));
    end
    if i==1
        addCollision(robot.Base,colLink{i});    
    else
        addCollision(robot.Bodies{i-1},colLink{i});    
    end    
end

end

%
% rot.m 
%
% rotation matrix about vector k over angle theta
% 
% R=rot(k,theta)
%

function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

%
% q=subprob0(k,p1,p2)
%
% solve for q subtended between p1 and p2
%    k determines the sign of q
%
% input: k,p1,p2 as R^3 column vectors
% output: q (scalar)
%

function q=subprob0(k,p1,p2)

if ((k'*p1)>sqrt(eps)|(k'*p2)>sqrt(eps))
  error('k must be perpendicular to p and q');
end

p1=p1/norm(p1);
p2=p2/norm(p2);

q=2*atan2(norm(p1-p2),norm(p1+p2));

if k'*(cross(p1,p2))<0
  q=-q;
end

end

