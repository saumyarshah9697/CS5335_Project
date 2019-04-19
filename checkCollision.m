function collision = checkCollision(rob,q,sphereCenter,r)

    % get points on elbow and at EE
    x1 = [0;0;0];
    
    T3 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q);
    x3 = T3(1:3,4);
    T4 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q);
    x4 = T4(1:3,4);
    T5 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q) * rob.A(5,q);
    x5 = T5(1:3,4);
    T6 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q) * rob.A(5,q)* rob.A(6,q);
    x6 = T6(1:3,4);
    
    
    vec = 0:0.1:1;
    m = size(vec,2);
    
    x13 = repmat(x3-x1,1,m) .* repmat(vec,3,1) + repmat(x1,1,m);
    x34 = repmat(x4-x3,1,m) .* repmat(vec,3,1) + repmat(x3,1,m);
    x45 = repmat(x5-x4,1,m) .* repmat(vec,3,1) + repmat(x4,1,m);
    x56 = repmat(x6-x5,1,m) .* repmat(vec,3,1) + repmat(x5,1,m);
    x = [x13 x34 x45 x56];
    
    if sum(sum((x - repmat(sphereCenter,1,size(x,2))).^2,1) < r^2) > 0
        collision = 1;
    else
        collision = 0;
    end




end