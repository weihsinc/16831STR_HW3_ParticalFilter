function parseData(filename)
%%
fid = fopen(filename);

odoms = [];
lasers = [];
tline = fgets(fid);
while ischar(tline)
%     disp(tline)
    c = strsplit(tline);
    if strcmp(c{1}, 'L')
        for i = 2:4
            odoms = [odoms str2num(c{i})];
        end
        
        for i = 8:187
            lasers = [lasers str2num(c{i})];
        end
    end
    tline = fgets(fid);
end

fclose(fid);

odoms = reshape(odoms, [3, length(odoms)/3]);
lasers = reshape(lasers,[180, length(lasers)/180]);

N = size(odoms,2);
% figure
% plot(odoms(1,:), odoms(2,:), '.')
% axis equal

%%
% figure
sim_x = []; sim_y = [];
for n = 1:N
n
% plot(odoms(1,n), odoms(2,n), 'bx'); hold on;

for i = 1:180
th = odoms(3,n) - pi/2 + i*pi/180;
x = lasers(i,n)*cos(th) + odoms(1,n);
y = lasers(i,n)*sin(th) + odoms(2,n);
sim_x = [sim_x x]; sim_y = [sim_y y];
% plot(x,y,'r.'); hold on
end
% axis equal
% pause(0.1)
end

sim_x = reshape(sim_x, [180, N]);
sim_y = reshape(sim_y, [180, N]);
save([filename(1:end-4), '.mat'], 'odoms', 'lasers', 'sim_x', 'sim_y');

end