clear all; close all; clc;
robotstart = [10 10];
sources = [350, 340; 150, 350; 100, 100];
obstacles = (imread('map1.png')==0);
obstacles = obstacles(:, :, 1);

iter = 20; %steps for contamination spread

figure('units','normalized','outerposition',[0 0 1 1]);
figure(1);
imagesc(obstacles'); axis square; colorbar; colormap jet; hold on;
figure(2);
imagesc(obstacles'); axis square; colorbar; colormap jet; hold on;

contam = zeros(size(obstacles));

maxTime = 5;
for i=1:maxTime
    contam = updatePol(contam,obstacles,sources, iter);
    figure(2);
    imagesc(contam');
    pause(0.1);
end

%plot dist
c1 = contam(1:200, 1:200);
[x, y] = meshgrid(1:200, 1:200);
x = (x-100).^2;
y = (y-100).^2;
dis = sqrt(x+y);

c1_lin = reshape(c1, [200*200, 1]);
dis_lin = reshape(dis, [200*200, 1]);
scatter(dis_lin, c1_lin);
xlabel('distance from source (px)');
ylabel('contamination');

yfit = 0:0.00005:0.012;
%xfit = -28*log(yfit/0.014);
hold on;
plot(xfit, yfit);

