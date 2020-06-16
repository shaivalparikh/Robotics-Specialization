function route = DijkstraTorus (input_map, start_coords, dest_coords)
    % Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0];

colormap(cmap);

label = true;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distances = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distances(start_node) = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    %image(1.5, 1.5, map);
    %grid on;
    %axis image;
    %drawnow;
%     
    % Find the node with the minimum distance
    [min_dist, current] = min(distances(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distances(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distances), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
   
    %%% All of your code should be between the two lines of stars. 
    % *******************************************************************
     if (i>1 && i<=nrows)
        ii = i-1;
        jj = j;
        update (ii,jj,min_dist+1,current);
    end
    
    if i<=1
        ii = nrows;
        jj = j;
        update (ii-1,jj,min_dist+1,current);
    end
    
    if (i>=1 && i<nrows)
        ii = i+1;
        jj = j;
        update (ii,jj,min_dist+1,current);
    end
    
    if i>=nrows
        ii = 1;
        jj = j;
        update (ii+1,jj,min_dist+1,current);
    end
    
    if (j>1 && j<=ncols)
        jj = j-1;
        ii = i;
        update (ii,jj,min_dist+1,current);
    end
    
    if j<=1
        ii = i;
        jj = ncols;
        update (ii,jj-1,min_dist+1,current);
    end
    
    if (j>=1 && j<ncols)
        jj =j+1;
        ii = i;
        update (ii,jj,min_dist+1,current);
    end
    
    if j>=ncols
        ii = i;
        jj = 1;
        update (ii,jj+1,min_dist+1,current);
    end 
    % *******************************************************************
end

if (isinf(distances(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    drawMap(label);
end
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

    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (distances(i,j) > d) )
            distances(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

    function drawMap(label)
        if label==true
        for k = 2:length(route) - 1        
            map(route(k)) = 7;
        end
        image(1.5, 1.5, map);
        grid on;
        axis image;
        end
        end
end