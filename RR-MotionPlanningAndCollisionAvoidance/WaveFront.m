function [wPath] = WaveFront(configurationSpace, alpha, beta, a2, b2, a1, b1, wSteps)

    wavefront = configurationSpace;
    wavefront(a1,b1) = 2;
    adj = [0 -1; -1 0; 1 0; 0 1];
    i_list = [a1, b1];

    while size(i_list,1) ~= 0
        % Iterate through cells adjacent to the cell at the top of the open queue:
        for k=1:size(adj,1)
            % Calculate index for current adjacent cell:
            cur_adj = i_list(1,:)+adj(k,:);
            % Make sure adjacent cell is in the map
            if min(cur_adj) < 1 || cur_adj(1) > length(alpha) || cur_adj(2) > length(beta)
              continue
            end
            % Make sure the adjacent cell is not an obstacle 
            if configurationSpace(cur_adj(1), cur_adj(2)) == 1 
                wavefront(cur_adj(1), cur_adj(2)) = 1;
                continue
            end

            % or not iterated
            if wavefront(cur_adj(1), cur_adj(2)) ~= 0 && wavefront(cur_adj(1), cur_adj(2)) ~= 1.5 
                continue
            end
            % Set the cost and add the adjacent to the open set
            wavefront(cur_adj(1), cur_adj(2)) = wavefront(i_list(1,1), i_list(1,2)) + 1;
            i_list(size(i_list,1)+1,:) = cur_adj;
        end

        % Pop the top open cell from the queue
        i_list = i_list(2:end,:);
    end

    % Find a path to the goal.
    wPath = [a2; b2];
    for j = 1:wSteps-1

        i = size(wPath,2);
        if wPath(:,i) == [a1; b1];
            break
        end

        curc = wavefront(wPath(1,j), wPath(2,j));
        if curc == 1 || curc == 1.5
            % if we're in an obstacle (bad initial state, most likely)
            curc = max(wavefront);
        end

        noMove = 1;    
        for k=1:size(adj,1)
            % Calculate index for current adjacent cell:
            cur_adj = wPath(:,j)+adj(k,:)';
            % Make sure adjacent cell is in the map
            if min(cur_adj) < 1 || cur_adj(1) > length(alpha) || cur_adj(2) > length(beta)
              continue
            end

            % If this adjacent cell reduces cost, add it to the path.
            if wavefront(cur_adj(1),cur_adj(2)) == 1
              % (obstacle)
              continue;
            end
            if wavefront(cur_adj(1),cur_adj(2)) < curc
              noMove = 0;
              wPath(:,j+1) = cur_adj;
              break
            end
        end
        if noMove
            wPath(:,j+1) = [a2; b2];
            break
        end
    end
    
end