
fig_num = 23333;
figure(fig_num);
clf;
hold on;
axis equal;
grid on; grid minor;

distance = (1:100)';
IR_distance = fcn_emulateIRSensorDistance(distance);
plot(distance,IR_distance,'k')

