% initialization
clear all; close all;
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% syms t1 t2 real;
%syms t1 real;
%t2 = acos(-1/3);

% R1 = rot(-ex,t1);
% R2 = rot([-sin(pi/6); cos(pi/6); 0],t2);
% 
% R = R1*R2;
% pretty(simplify(R))

%[q1 q2] = subprob2(ex,ez,[-sin(pi/6); cos(pi/6); 0],ex)
[q1 q2] = subprob2(ez,ex,ex,[-sin(pi/6); cos(pi/6); 0])

rot(-ex,q1(1))*rot([-sin(pi/6); cos(pi/6); 0],q2(1))*ex


function R=rot(k,theta)
    k=k/norm(k);
    R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
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

function q_inpi=inpi(q)
    while q > pi
        q = q-2*pi;
    end
    while q < -pi
        q = q+2*pi;
    end
    q_inpi = q;
end