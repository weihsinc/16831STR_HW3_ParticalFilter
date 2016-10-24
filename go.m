close all
clear all
%%
filename = 'data/log/robotdata1';
% parseData([filename, '.log'])
%%
load([filename, '.mat']);
N = length(odoms);

figure
plot(odoms(1,:), odoms(2,:), '.')
axis equal


animate = true
figure
for n = 1:N

plot(odoms(1,n), odoms(2,n), 'bx'); hold on;
plot(sim_x(:,n),sim_y(:,n),'r.'); hold on;
axis equal;
% axis([odoms(1,n)-1000, odoms(1,n)+1000, odoms(2,n)-1000, odoms(2,n)+1000]);
if animate
    pause(0.1)
end

end

figure
plot(odoms(1,:), odoms(2,:), 'bx'); hold on;
plot(sim_x,sim_y,'r.'); hold on;