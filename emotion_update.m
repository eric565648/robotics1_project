function param_desired = emotion_update(target_score,origin_param,learn_coeff,learn_inter,std_mean,std_scale)

    param_std = (origin_param-std_mean)./std_scale;
    % xd = x+((yd-bo-b^Tx)/b^T*b)*b (xd=x+alpha*beta)
    param_desired_std = param_std+((target_score-learn_inter-learn_coeff'*param_std)/(learn_coeff'*learn_coeff))*learn_coeff;

    param_desired = param_desired_std.*std_scale+std_mean;
end

