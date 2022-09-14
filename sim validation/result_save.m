times = out.da.time;
das = out.da.signals.values;
pos = out.pos.signals.values;
save('data2.mat', 'heights', 'wind_profile_hat', 'delta_ws','times', 'das', 'pos')

