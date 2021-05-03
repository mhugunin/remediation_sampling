function[numofmoves, caught] = runtest(robotstart, mapfile)

close all;

if exist('mapfile', 'var')
    map_struct = load(mapfile);
    envmap = map_struct.contam;
    obsmap = map_struct.obstacles;
    sources = map_struct.sources;
else
    %TODO: should we specify target location?
    [envmap, obsmap, sources] = generate_map(0, 0);
end

%draw the environment
fig = figure('units','normalized','outerposition',[0 0 1 1]);
sp1 = subplot(2,2,1);
sp1.Position = sp1.Position + [0.075 -0.05 0 0.1];
imshow(ones(size(envmap')));
sp2 = subplot(2,2,2);
sp2.Position = sp2.Position + [-0.075 -0.05 0 0.1];
imshow(ones(size(envmap')));
sp3 = subplot(2,2,3);
sp3.Position = sp3.Position + [0.075 -0.05 0 0.1];
imshow(ones(size(envmap')));
sp4 = subplot(2,2,4);
sp4.Position = sp4.Position + [-0.075 -0.05 0 0.1];
imshow(ones(size(envmap')));
% imagesc(envmap'); axis square; colorbar; colormap jet; hold on;
% imagesc(obsmap'); axis square; colorbar; colormap jet;

% show obstacles and contamination on same image
imInd = gray2ind(envmap', 256);
rgbImage = ind2rgb(imInd, jet(256));
rgbImage(:,:,:) = rgbImage(:,:,:) .* ~obsmap(:,:)';
axes(sp1);
imshow(rgbImage);
hold on;

exploredmap = zeros(size(envmap));
exploredmap = logical(exploredmap);
exploredmap(robotstart(1), robotstart(2)) = 1;

goalmap = ones(size(envmap)) / (size(envmap, 1) * size(envmap, 2));
%current positions of the target and robot
robotpos = robotstart;

%now comes the main loop
hr = -1;
goal_guess_plot = -1;
goal_guess_plot2 = -1;
numofmoves = 0;
caught = 0;
%c0 = clock();

%meshgrid for distance calcs
[x, y] = meshgrid(1:size(envmap, 1), 1:size(envmap, 2));

for i = 1:1000
    %draw the positions
    if (hr ~= -1)
        delete(hr);
    end
    figure(1);
    axes(sp1);
    hr = text(robotpos(1), robotpos(2), 'R', 'Color', 'w', 'FontWeight', 'bold');
    hr0 = scatter(robotpos(1), robotpos(2), 10, 'w', 'filled');
    
    axes(sp2);
    imshow((exploredmap.*~obsmap + 0.5*(~exploredmap))');
    
    axes(sp3);
    imshow(goalmap/max(max(goalmap)));
    hold on;
    [~, n] = max(goalmap(:));
    [x_max, y_max] = ind2sub(size(goalmap), n);
    axes(sp1);
    if (goal_guess_plot ~= -1)
        delete(goal_guess_plot);
    end
    goal_guess_plot = scatter(y_max, x_max, 'm', 'LineWidth', 2);  
    axes(sp3);
    if (goal_guess_plot2 ~= -1) 
        delete(goal_guess_plot2);
    end
    goal_guess_plot2 = scatter(y_max, x_max, 'm', 'LineWidth', 2);
    hold off;
    
    %call robot planner to find what they want to do
    %tStart = tic;
    % want our planner to return entire path to next frontier location
    localplan = robotplanner(envmap, obsmap, exploredmap, [y_max x_max], robotpos);
%     localplan = [robotpos(1)-1 robotpos(2)-1; robotpos(1)-2, robotpos(2)-2]; %TEMP STAND-IN
%     
%     if i > 20
%         localplan = [robotpos(1)-1 robotpos(2); robotpos(1)-2, robotpos(2)];
%     end
    
    %compute movetime for the target
    %tElapsed = toc(tStart);
    %timeTaken = tElapsed*1000; % in ms
    %movetime = max(1, ceil(timeTaken/200));
    
    for j = 1:size(localplan, 1)
        newrobotpos = localplan(j, :);
        %check that the new commanded position is valid
        if (newrobotpos(1) < 1 || newrobotpos(1) > size(envmap, 1) || ...
                newrobotpos(2) < 1 || newrobotpos(2) > size(envmap, 2))
            fprintf(1, 'ERROR: out-of-map robot position commanded\n');
            return;
        elseif (obsmap(newrobotpos(1), newrobotpos(2)) ~= 0)
            fprintf(1, 'ERROR: invalid robot position commanded\n');
            return;
        elseif (abs(newrobotpos(1)-robotpos(1)) > 1 || abs(newrobotpos(2)-robotpos(2)) > 1)
            fprintf(1, 'ERROR: invalid robot move commanded\n');
            return;
        end        

        %make the moves
        robotpos = double(newrobotpos);
        % exploredmap(robotpos(1), robotpos(2)) = 1;
        numofmoves = numofmoves + 1;
        
        pause(0.1);
    end
    
    %Update contamination map
    contam_reading = envmap(robotpos(1), robotpos(2));
    exp_dist = 35*((-1*log(contam_reading)).^(3/4));
    
    dis = sqrt((x-robotpos(1)).^2+(y-robotpos(2)).^2);
    c = 5+15*(1-contam_reading);
    
    p_gau_far = 1+(contam_reading*(exp(-1*(dis-exp_dist).^2/(2*c^2))-1));
    p_gau_near = 1+(contam_reading*(exp(-1*(dis-exp_dist).^2/(2.5*2*c^2))-1));
    p_gau = p_gau_far.*(dis>exp_dist)+p_gau_near.*(dis<=exp_dist);
    axes(sp4);
    imshow(p_gau);
%     pause(0.1);
    
    goalmap = goalmap.*p_gau;
    goalmap = goalmap ./ sum(sum(goalmap)); %normalize probs to sum of 1
    
    %TODO: Implement stopping condition -> should be signaled by the
    %planner when it thinks it found the source
    %check if target is caught
%     for source_ind = 1:size(sources, 1)
%          if (abs(robotpos(1)-sources(source_ind, 1)) <= 1 && abs(robotpos(2)-sources(source_ind, 2)) <= 1)
%              caught = 1;
%              break; %Condition will be different if multigoal
%          end
%     end
    if (abs(robotpos(1) - y_max) <= 1 && abs(robotpos(2) - x_max) <= 1)
        caught = 1;
        break;
    end
end

fprintf(1, 'Robot stopped=%d, number of moves made=%d\n', caught, numofmoves);
%c = clock();
%fprintf(1, 'duration=%d\n', (c(5) - c0(5)) * 60 + c(6) - c0(6));
