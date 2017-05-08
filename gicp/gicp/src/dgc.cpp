     ps_out.AlignScan(&ps_in, T_guess, T_delta, 5); 
     dgc_transform_copy(T_est, T_guess); 
     dgc_transform_left_multiply(T_est, T_delta);     
