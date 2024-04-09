function [pt_nearest, d_nearest] = findNearestPtInSetsL1(sets, pt_sample)
    % Find the closest point in the sets to the sample
    % use the L1 norm as distance metric
    points      = zeros(length(pt_sample), length(sets));
    distances   = zeros(1, length(sets));
    for i = 1:length(sets)
        [pt, distance]  = findNearestPtInSetL1(sets{i}, pt_sample);
        points(:,i)     = pt;
        distances(i)    = distance;
    end
    [~, ith_pt] = min(distances);
    pt_nearest  = points(:,ith_pt);
    d_nearest   = distances(ith_pt);
end

function [pt_nearest, distance] = findNearestPtInSetL1(set, pt_sample)
    % Find the closest point in the set to the sample
    % use the L1 norm as distance metric
    if in(set, pt_sample) == 1
        pt_nearest  = pt_sample;
        distance    = 0;
    else
        pt_nearest  = NaN(length(pt_sample), 1);
        for x_i = 1:length(pt_sample)
            if pt_sample(x_i) <= set.inf(x_i)
                pt_nearest(x_i)     = set.inf(x_i);
            elseif pt_sample(x_i) >= set.sup(x_i)
                pt_nearest(x_i)     = set.sup(x_i);
            else
                pt_nearest(x_i)     = pt_sample(x_i);
            end
        end
        distance    = norm(pt_nearest-pt_sample, 1);
    end
end