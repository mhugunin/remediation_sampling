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

maxTime = 500;
for i=1:maxTime
    contam = updatePol(contam,obstacles,sources, iter);
    figure(2);
    imagesc(contam');
    pause(0.1);
end
