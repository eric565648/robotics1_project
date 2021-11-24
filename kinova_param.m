% parameters
wheel_r = 0.127;
d1 = 0.2755;
l1=0.3; l2=wheel_r+0.3+d1;
d2 = 0.41; d3=0.2073; e2=0.0098; d4=0.0741; d5=d4; d6=0.16;
aa = pi/6;
p89x = d4*(sin(aa)/sin(2*aa)) + 2*d4*(sin(aa)/sin(2*aa))*cos(2*aa);
p89y = 2*d4*sin(aa);

% POE robot_arm def
armbot.H = [ez ey -ey -ex [-sin(pi/6); cos(pi/6); 0] -ex];
armbot.P = [d1*ez 0*ez d2*ez+e2*ey 0*ex (p89x+d3)*ex-p89y*ey 0*ex (d6+d4*sin(aa)/sin(2*aa))*ex];
armbot.joint_type = [0 0 0 0 0 0];
armbot.joint_upper_limit = deg2rad([120 130 180 10000 10000 10000]);
armbot.joint_lower_limit = deg2rad([-120 -130 -71 -10000 -10000 -10000]);
armbot.home = [0 0 0 pi/6 -pi/6 0];
armbot.P4T = (p89x+d3)*ex;