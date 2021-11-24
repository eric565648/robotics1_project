% initialization
clear all; close all;

% a2=0.4100;
% b1=0.2102; b3=-0.0120; b4=-0.2493; b5=-0.0846; b6=-0.2273;
% t1=pi/2; t2=pi; t3=pi/2; t4=pi*55/180; t5=t4; t6=pi;

syms a2 b1 b3 b4 b5 b6 t1 t2 t3 t4 t5 t6 real;
% 
syms q11 q12 q13 q21 q22 q23 q31 q32 q33 real;
syms r1 r2 r3 real;

syms s1 c1 s4 real;

% c4 = 1/(b5*sin(t4))*(r1*s1-r2*c1+b3);
% A1 = cos(t4)*(q13*c1+q23*s1)-sin(t4)*q33*s4;
% A2 = sin(t4)*(q13*c1+q23*s1)*s4+cos(t4)*q33;
% A3=cos(t5)-sin(t4)*(-q13*s1+q23*c1)*c4;
% 
% B1 = 2*(r3-b1)*b5*sin(t4)*s4 - 2*(r1*c1+r2*s1)*(b4+b5*cos(t4));
% B2 = -2*(b4+b5*cos(t4))*(r3-b1)-2*(r1*c1+r2*s1)*b5*sin(t4)*s4;
% B3 = (r3-b1)^2 + (r1*c1+r2*s1)^2+(b4+b5*cos(t4))^2+b5^2*(sin(t4))^2*s4^2-a2^2;
% 
% f = (A2*B3-A3*B2)^2+(A3*B1-A1*B3)^2-(A1*B2-B1*A2)^2;
%pretty(simplify(expand(f)));
%pretty(expand(f))
% f = collect(f,s4);
% f = subs(f,s4^2,1-c4^2);
% f = subs(f,s4^3,(1-c4^2)*s4);
% f = subs(f,s4^4,(1-c4^2)^2);
% f = collect(f,s4);

V = (2*(q33*cos(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - (cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(b1 - r3)*(2*b4 + 2*b5*cos(t4)))*(sin(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1)) - 2*(q33*sin(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*b1 - 2*r3))*(cos(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + (b4 + b5*cos(t4))*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1)) - ((b3 - c1*r2 + r1*s1)^2/(b5^2*sin(t4)^2) - 1)*(2*(b5*q33*sin(t4)^2*(2*c1*r1 + 2*r2*s1) + b5*sin(t4)^2*(c1*q13 + q23*s1)*(2*b1 - 2*r3))*(q33*sin(t4)*(b1 - r3)*(2*b4 + 2*b5*cos(t4)) - sin(t4)*(b4 + b5*cos(t4))*(c1*q13 + q23*s1)*(2*c1*r1 + 2*r2*s1) + b5*cos(t4)*sin(t4)*(c1*q13 + q23*s1)*(2*c1*r1 + 2*r2*s1) - b5*q33*cos(t4)*sin(t4)*(2*b1 - 2*r3)) - 2*b5^2*q33*sin(t4)^3*(cos(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + (b4 + b5*cos(t4))*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1)) + 2*b5^2*sin(t4)^3*(q33*cos(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - (cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(b1 - r3)*(2*b4 + 2*b5*cos(t4)))*(c1*q13 + q23*s1) + 2*b5^2*q33*cos(t4)*sin(t4)^2*(sin(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1)) - 2*b5^2*cos(t4)*sin(t4)^2*(q33*sin(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*b1 - 2*r3))*(c1*q13 + q23*s1)) + 2*(cos(t4)*(b1 - r3)*(2*b4 + 2*b5*cos(t4))*(c1*q13 + q23*s1) + q33*cos(t4)*(b4 + b5*cos(t4))*(2*c1*r1 + 2*r2*s1))*(q33*sin(t4)*(b1 - r3)*(2*b4 + 2*b5*cos(t4)) - sin(t4)*(b4 + b5*cos(t4))*(c1*q13 + q23*s1)*(2*c1*r1 + 2*r2*s1) + b5*cos(t4)*sin(t4)*(c1*q13 + q23*s1)*(2*c1*r1 + 2*r2*s1) - b5*q33*cos(t4)*sin(t4)*(2*b1 - 2*r3)));
W = (cos(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + (b4 + b5*cos(t4))*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1))^2 - ((b3 - c1*r2 + r1*s1)^2/(b5^2*sin(t4)^2) - 1)*((sin(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1))^2 - (q33*sin(t4)*(b1 - r3)*(2*b4 + 2*b5*cos(t4)) - sin(t4)*(b4 + b5*cos(t4))*(c1*q13 + q23*s1)*(2*c1*r1 + 2*r2*s1) + b5*cos(t4)*sin(t4)*(c1*q13 + q23*s1)*(2*c1*r1 + 2*r2*s1) - b5*q33*cos(t4)*sin(t4)*(2*b1 - 2*r3))^2 - 2*(b5*q33*sin(t4)^2*(2*c1*r1 + 2*r2*s1) + b5*sin(t4)^2*(c1*q13 + q23*s1)*(2*b1 - 2*r3))*(cos(t4)*(b1 - r3)*(2*b4 + 2*b5*cos(t4))*(c1*q13 + q23*s1) + q33*cos(t4)*(b4 + b5*cos(t4))*(2*c1*r1 + 2*r2*s1)) + (q33*sin(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*b1 - 2*r3))^2 + 2*b5^2*q33*cos(t4)*sin(t4)^2*(q33*cos(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - (cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(b1 - r3)*(2*b4 + 2*b5*cos(t4))) + 2*b5^2*cos(t4)*sin(t4)^2*(cos(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + (b4 + b5*cos(t4))*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1))*(c1*q13 + q23*s1)) - (b5^4*q33^2*sin(t4)^6 + b5^4*sin(t4)^6*(c1*q13 + q23*s1)^2)*((b3 - c1*r2 + r1*s1)^2/(b5^2*sin(t4)^2) - 1)^3 + ((b3 - c1*r2 + r1*s1)^2/(b5^2*sin(t4)^2) - 1)^2*(2*b5^2*sin(t4)^3*(sin(t4)*(c1*q13 + q23*s1)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) + b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*c1*r1 + 2*r2*s1))*(c1*q13 + q23*s1) - (b5*q33*sin(t4)^2*(2*c1*r1 + 2*r2*s1) + b5*sin(t4)^2*(c1*q13 + q23*s1)*(2*b1 - 2*r3))^2 + 2*b5^2*q33*sin(t4)^3*(q33*sin(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - b5*sin(t4)*(cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(2*b1 - 2*r3)) + b5^4*q33^2*cos(t4)^2*sin(t4)^4 + b5^4*cos(t4)^2*sin(t4)^4*(c1*q13 + q23*s1)^2) + (q33*cos(t4)*((c1*r1 + r2*s1)^2 + (b4 + b5*cos(t4))^2 - a2^2 + (b1 - r3)^2) - (cos(t5) - ((c1*q23 - q13*s1)*(b3 - c1*r2 + r1*s1))/b5)*(b1 - r3)*(2*b4 + 2*b5*cos(t4)))^2 - (cos(t4)*(b1 - r3)*(2*b4 + 2*b5*cos(t4))*(c1*q13 + q23*s1) + q33*cos(t4)*(b4 + b5*cos(t4))*(2*c1*r1 + 2*r2*s1))^2;

V = expand(V);
V = subs(V,s1^5,(1-c1^2)^2*s1);
V = subs(V,s1^4,(1-c1^2)^2);
V = subs(V,s1^3,(1-c1^2)*s1);
V = subs(V,s1^2,(1-c1^2));
V = expand(V);
W = expand(W);
W = subs(W,s1^5,(1-c1^2)^2*s1);
W = subs(W,s1^4,(1-c1^2)^2);
W = subs(W,s1^3,(1-c1^2)*s1);
W = subs(W,s1^2,(1-c1^2));
W = expand(W);

% simplify(V)

% V = collect(V,c1^3);
% V = collect(V,c1^2*s1);
% V = collect(V,c1^2);
% V = collect(V,c1*s1);
% V = collect(V,c1);
% V = collect(V,s1);
 V = collect(V,[c1 s1]);
W = collect(W,[c1 s1]);

syms T real;

% p = b5^2*sin(t4)^2*W^2 + ((r1*s1-r2*c1+b3)^2-b5^2*sin(t4)^2)*V^2;
% p = subs(p,c1,(1-T^2)/(1+T^2));
% p = subs(p,s1,2*T/(1+T^2));
% %p = p*(1+T^2);
% 
% p = expand(p);

% p
