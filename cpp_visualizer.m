traj = readtable('traj_log.csv'); 
x=traj{:,1};
y=traj{:,2};

figure; 
plot(x, y); 