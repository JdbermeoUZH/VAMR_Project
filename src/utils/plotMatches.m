function plotMatches(matches, query_keypoints, database_keypoints)

%[Matches] = 1 x number of keypoints
%[query_keypoints] = 2 x number of keypoints
%[database_keypoints] = 2 x number of keypoints


%[~, query_indices, match_indices] = find(matches);
query_indices = matches(:,2);
match_indices = matches(:,1);

%[query_indices] = 1 x 2, indices of matches ~=0
%[match_indices] = 1 x 2, values of matches ~=0

x_from = query_keypoints(1, query_indices);
x_to = database_keypoints(1, match_indices);
y_from = query_keypoints(2, query_indices);
y_to = database_keypoints(2, match_indices);
plot([y_from; y_to], [x_from; x_to], 'g-', 'Linewidth', 3);

end
