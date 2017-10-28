function draw_energy(data, limit)
data = data(1:limit,:);
figure;
h = area((1:1:limit) * 0.01, data);
h(1).FaceColor = [1 0 0];
h(2).FaceColor = [0 1 0];
xlim([0 limit * 0.01]);
legend('Kinetic Energy', 'Potential Energy');
set(gca,'FontSize',16);
grid on;
xlabel('t (s)');
ylabel('Energy (J)');
end