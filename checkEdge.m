function collision = checkEdge(rob,q1,q2,sphereCenter,r)

    n = 10; % Num pts along vector to check
    m = size(q1,2); % Dimension of c-space
    
    % Generate n pts between q1 and q2
    viaPts = repmat(q2-q1,[n,1]) .* repmat(linspace(0,1,n)', [1 m]) + repmat(q1,[n,1]);
    
    % Check those pts for collision
    collision = 0;
    for i = 1:n
        collision = robotCollision(rob,viaPts(i,:),sphereCenter,r);
        if collision
            break
        end
    end
    
end
