%% Finds the transformation from front to back cameras
% Assumes the points_front & points_back matrices are already
% populated

% Twelve parameters of the transformation matrix
big_cols = 13;
num_pts = -1;
big_rows = -1;

% Rows depends on the number of points -- each point 
% generates three equations
if (size(points_front,1) == size(points_back,1))
    num_pts = size(points_front,1);
    big_rows = 3 * size(points_front,1);
else
    'Number of points in front and back should always match..'
end

big_mat = zeros(big_rows, big_cols);

% for each point
for i = 1:num_pts
    cur_f_pt = points_front(i,:);
    cur_b_pt = points_back(i,:);
    % for each equation
    for j = 1:3
        cur_row = (i-1)*3 + j;
        for k = 1:4
            cur_col = (j-1) * 4 + k;
            if k ~= 4
                big_mat(cur_row, cur_col) = cur_f_pt(k);
            else
                big_mat(cur_row, cur_col) = 1;
            end
        end
        big_mat(cur_row, end) = -cur_b_pt(j);
    end
end

%Debug
%big_mat

% DLT
[u, s, v] = svd(big_mat);
trans = v(:,end);
transform = trans(1:end-1,:)/trans(end,:);
transform = (reshape(transform, 4, 3))';
transform = [transform; 0 0 0 1];
% debug
transform

