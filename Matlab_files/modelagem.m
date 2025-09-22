data = readmatrix('Log.csv');   


t = data(:,1);
v = data(:,2);
u = data(:,3);

t_filt = t(12:end);
v_filt = v(12:end);
u_filt = u(12:end);
