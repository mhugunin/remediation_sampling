function contam = updatePol(contam,obstacles,sources,iter, time)
%Updates polutant in a map
    convsize = 35;
    middle = double(uint8(convsize/2));
    % carr is noisy radial gradient kernel?
    carr = rand(convsize, convsize)*0.3+0.7;
    
    for i = 1:convsize
        for j = 1:convsize
            disx = abs(i-1.0*middle);
            disy = abs(j-1.0*middle);
            scale = norm([disx, disy])/(middle);
            scale = 1.0-scale;
            scale = max(0, scale);
            carr(i, j)=carr(i, j)*scale;
        end
    end
    carr = carr/sum(sum(carr));
    
    for i = 1:iter
        for j = 1:size(sources, 1)
            contam(sources(j, 1), sources(j, 2)) = contam(sources(j, 1), sources(j, 2)) + 1.0;
            %[sources(j, 1), sources(j, 2)]
        end
                
        %conv
        contam = conv2(contam, carr, 'same');
        
        %normalize / total contamination should make sense
        contam = (contam .* ~obstacles);
        contam = (((time-1)* iter * size(sources, 1)) + i * size(sources, 1)) * contam / sum(sum(contam));
        
    end
end

