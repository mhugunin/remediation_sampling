function [contam, obstacles, sources] = generate_map(display, save_maps)

if display
    close all;
end

% randomize source location(s)?
% make number of sources a parameter?
sources = [268, 283];
obstacles = (imread('map1.png')==0);
obstacles = obstacles(:, :, 1);

iter = 20; %steps for contamination spread

if display
    figure('units','normalized','outerposition',[0 0 1 1]);
    figure(1);
    imagesc(obstacles'); axis square; colorbar; colormap jet; hold on;
    figure(2);
    imagesc(obstacles'); axis square; colorbar; colormap jet; hold on;
end

contam = zeros(size(obstacles));

maxTime = 3;
for i=1:maxTime
    contam = updatePol(contam,obstacles,sources, iter, i);
    if display
        figure(2);
        imagesc(contam' / max(max(contam)));
        pause(0.1);
    end
end

contam = contam / max(max(contam));

if save_maps
    save('map', 'contam', 'obstacles', 'sources');
end

%plot dist
if display
    figure()
    x_range = max(sources(1,1)-99, 1):min(sources(1,1)+100, size(contam, 1));
    y_range = max(sources(1,2)-99, 1):min(sources(1,2)+100, size(contam, 2));
    c1 = contam(x_range, y_range);
    [x, y] = meshgrid(x_range, y_range);
    x = (x-sources(1,1)).^2;
    y = (y-sources(1,2)).^2;
    dis = sqrt(x+y);

    c1_lin = reshape(c1, [length(x_range)*length(y_range), 1]);
    dis_lin = reshape(dis', [length(x_range)*length(y_range), 1]);
    scatter(dis_lin, c1_lin);
    xlabel('distance from source (px)');
    ylabel('contamination');

    yfit = 0:0.001:1;
    xfit = 35*((-1*log(yfit)).^(3/4));
    hold on;
    plot(xfit, yfit);
end

end
