close all;
clear all;

oarbot_param;

kinova_param;

dataN = 5;
target_score=75;
learn_coeff_std = readmatrix('data/learn_coeff.csv','Delimiter',',');
learn_coeff = learn_coeff_std(3,:);
std_mean = learn_coeff_std(1,:);
std_scale = learn_coeff_std(2,:);
learn_coeff_intercept = readmatrix('data/learn_coeff_intercept.csv','Delimiter',',');

param_upper_limit = [0.65 deg2rad(150-60) deg2rad(180-90) 0.3 1 0.65 deg2rad(150-60) deg2rad(180-90) 0.3 1 -1 -1 0];
param_lower_limit = [0.3 -deg2rad(60) -deg2rad(90) 0 0 0.3 -deg2rad(60) -deg2rad(90) 0 0 -2 -2 -2];

paramM=[];

i=1;
while i<=dataN

    sr = rand*0.35 + 0.3;stheta = rand*deg2rad(150)-deg2rad(60);
    sp = rand*deg2rad(180)-deg2rad(90);sh = rand*0.3; shand=rand;
    er = rand*0.35 + 0.3;etheta = rand*deg2rad(150)-deg2rad(60);
    ep = rand*deg2rad(180)-deg2rad(90);eh = rand*0.3; ehand=rand;
    exp_r = rand-2;
    arm_speed = rand-2;
    vel_type=randi([1,6]);
    base_speed=rand*2-2;
    
    param = [sr stheta sp sh shand er etheta ep eh ehand exp_r arm_speed base_speed];
    updated_param=emotion_update(target_score,param',learn_coeff',learn_coeff_intercept',std_mean',std_scale');
    updated_param = updated_param';
    if (sum(updated_param>param_upper_limit)>0) 
        disp("Higher than bound")
        disp(updated_param)
        disp(param_upper_limit)
        continue
    elseif (sum(updated_param<param_lower_limit)>0)
        disp("Lower than bound")
        disp(updated_param)
        disp(param_lower_limit)
        continue
    end
    
    disp("Chosen one");
    disp(updated_param);
    disp(emotion_predict(updated_param',learn_coeff',learn_coeff_intercept',std_mean',std_scale'));
    
    sr=updated_param(1);stheta=updated_param(2);sp=updated_param(3);
    sh=updated_param(4);shand=updated_param(5);
    er=updated_param(6);etheta=updated_param(7);ep=updated_param(8);
    eh=updated_param(9);ehand=updated_param(10);
    exp_r=updated_param(11);
    arm_speed=updated_param(12);base_speed=updated_param(13);

    [path,pathp_d,pathq_d] = create_path(robot,armbot,sr,stheta,sp,sh,shand,er,etheta,ep,eh,ehand,exp_r,vel_type,arm_speed,base_speed);

    if ~isempty(path)
%         tic
%         for k=1:length(path(1,:))
%             qbot=path(1:end-1,k);
%             figure(1);show(robot_tree,qbot,'collision','on','PreservePlot',0,'FastUpdate',1);
%             view(43,16);axis([0 1 -0.5 0.5 0 2]);
%         end
%         toc
%         disp(length(path(1,:))*0.02)
        paramM = [paramM;sr,stheta,sp,sh,shand,er,etheta,ep,eh,ehand,exp_r,vel_type,arm_speed,base_speed];
        writematrix(path,'happy_motion/'+string(i)+'.csv');
        fprintf('Number %s\n',num2str(i));

%         figure(i);plot(vecnorm(diff(pathp_d')')/0.02);title('Velocity Type '+string(i));
%         ylabel('Path Speed m/s'); xlabel('Time Index'); grid;
        i=i+1;
%         plotvel = [plotvel;vecnorm(diff(pathp_d')')/0.02];
    end
end
writematrix(paramM,'happy_motion/happy_path_param.csv');