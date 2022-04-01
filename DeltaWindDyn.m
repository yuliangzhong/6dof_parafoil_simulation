function delta_w_dot = DeltaWindDyn(w)
    alpha_c = -0.004;
    beta_c = 0.02;
    delta_w_dot = alpha_c * w + beta_c * randn(2,1);
end