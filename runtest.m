function[numofmoves, caught] = runtest(robotstart, mapfile)

close all;

if exist('mapfile', 'var')
    map_struct = load(mapfile);
    envmap = map_struct.contam;
    obsmap = map_struct.obstacles;
else
    %TODO: should we specify target location?
    [envmap, obsmap] = generate_map(0, 0);
end

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
% imagesc(envmap'); axis square; colorbar; colormap jet; hold on;
% imagesc(obsmap'); axis square; colorbar; colormap jet;

% show obstacles and contamination on same image
imInd = gray2ind(envmap', 256);
rgbImage = ind2rgb(imInd, jet(256));
rgbImage(:,:,:) = rgbImage(:,:,:) .* ~obsmap(:,:)';
imshow(rgbImage);
hold on;

%current positions of the target and robot
robotpos = robotstart;

%now comes the main loop
hr = -1;
numofmoves = 0;
caught = 0;
c0 = clock();
for i = 1:2000

    %draw the positions
    if (hr ~= -1)
        delete(hr);
    end
    hr = text(robotpos(1), robotpos(2), 'R', 'Color', 'g', 'FontWeight', 'bold');
    hr = scatter(robotpos(1), robotpos(2), 10, 'g', 'filled');

    pause(0.1);
    %pause();
    
    %call robot planner to find what they want to do
    tStart = tic;
    envmap(1,1) = 2;
    newrobotpos = robotplanner(envmap, obsmap, robotpos);
    %compute movetime for the target
    tElapsed = toc(tStart);
    timeTaken = tElapsed*1000; % in ms
    
    movetime = max(1, ceil(timeTaken/200));
    
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
    robotpos = newrobotpos;
    numofmoves = numofmoves + 1;
    
    %TODO: Implement stopping condition -> should be signaled by the
    %planner when it thinks it found the source
    %check if target is caught
%     if (abs(robotpos(1)-targetpos(1)) <= 1 && abs(robotpos(2)-targetpos(2)) <= 1)
%         caught = 1;
%         break;
%     end
    
end

fprintf(1, 'Robot stopped=%d, number of moves made=%d\n', caught, numofmoves);
c = clock();
fprintf(1, 'duration=%d\n', (c(5) - c0(5)) * 60 + c(6) - c0(6));
