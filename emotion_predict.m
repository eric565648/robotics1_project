function score = emotion_predict(origin_param,learn_coeff,learn_inter,std_mean,std_scale)

    param_std = (origin_param-std_mean)./std_scale;
    score = learn_coeff'*param_std+learn_inter;
    
end

