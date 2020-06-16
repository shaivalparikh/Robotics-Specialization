function flag = triangle_intersection(P1,P2)
    flag = logical(1);
    axis1 = zeros(size(P1));
    axis2 = zeros(size(P2));
    for i = 1:length(P1(:,1))
        p1 = P1(i,:);
        if i ~= length(P1(:,1))
            p2 = P1(i+1,:);
        else
            p2 = P1(1,:);
        end
        edge = p1-p2;
        normal = [-edge(2) edge(1)]; % perpendicular to edge (normal)
        axis1(i,:) = normal / sqrt(normal*normal');
    end
    for i = length(P2(:,1))
        p1 = P2(i,:);
        if i ~= length(P2(:,1))
            p2 = P2(i+1,:);
        else
            p2 = P2(1,:);
        end
        edge = p1-p2;
        normal = [-edge(2) edge(1)]; % perpendicular to edge (normal)
        axis2(i,:) = normal / sqrt(normal*normal');
    end
    for i = 1:length(axis1(:,1))
        axis = axis1(i,:);
        min = P1(1,:)*axis';
        max = min;
        for j = 2:length(P1(:,1))
            p = P1(j,:)*axis';
            if (p < min)
                min = p;
            elseif (p > max)
                max = p;
            end
        end
        proj1.min = min;
        proj1.max = max;
        min = P2(1,:)*axis';
        max = min;
        for j = 2:length(P2(:,1))
            p = P2(j,:)*axis';
            if (p < min)
                min = p;
            elseif (p > max)
                max = p;
            end
        end
        proj2.min = min;
        proj2.max = max;
    
        if ( (proj1.max < proj2.min) || (proj2.max < proj1.min) )
        % we found a separating axis, they don't intersect
            flag = logical(0);
            return;
        end
    end
    for i = 1:length(axis2(:,1))
        axis = axis2(i,:);
        min = P1(1,:)*axis';
        max = min;
        for j = 2:length(P1(:,1))
            p = P1(j,:)*axis';
            if (p < min)
                min = p;
            elseif (p > max)
                max = p;
            end
        end
        proj1.min = min;
        proj1.max = max;
        min = P2(1,:)*axis';
        max = min;
        for j = 2:length(P2(:,1))
            axis = squeeze(axis1(j,:));
            p = P2(j,:)*axis';
            if (p < min)
                min = p;
            elseif (p > max)
                max = p;
            end
        end
        proj2.min = min;
        proj2.max = max;
    
        if ( (proj1.max < proj2.min) || (proj2.max < proj1.min))
        % we found a separating axis, they don't intersect
            flag = logical(0);
            return;
        end
    end
end
