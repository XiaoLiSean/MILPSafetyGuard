function trimedSets = trimUnsafeRegions(set, X_us)
    % Function used to trim unsafe region out of the set
    % Input: 
    %   set: interval that shall be trimed
    %   sets_unsafe: cell of intervals that is used to trim the set
    % Output:
    %   trimedSet: cell contains the resulted elementary boxes
    
    % Filter out sets that do not intersect with the set
    sets_unsafe     = {};
    for i = 1:length(X_us)
        if ~isempty(and(set, X_us{i}))
            sets_unsafe{end+1}  =  X_us{i};
        end
    end
    
    trimedSets  = {set};
    for i = 1:length(sets_unsafe)
        tmp_collection  = {};
        for j = 1:length(trimedSets)            
            trimedSet       = trimUnsafeRegion(trimedSets{j}, sets_unsafe{i});
            tmp_collection  = [tmp_collection(:); trimedSet(:)];
        end
        trimedSets  = tmp_collection;
    end
end

function trimedSet = trimUnsafeRegion(set, set_unsafe)
    % Function used to trim unsafe region out of the set
    % Input: 
    %   set: interval that shall be trimed
    %   set_unsafe: interval that is used to trim the set
    % Output:
    %   trimedSet: cell contains the resulted elementary boxes
    if isempty(and(set, set_unsafe))
        trimedSet   = {set};
    else
        % -----------------------------------------------------------------
        % The set is entirely unsafe
        % -----------------------------------------------------------------
        if in(set_unsafe, set)
            trimedSet   = {};
            return
        end
        % -----------------------------------------------------------------
        % Define cut lines
        % -----------------------------------------------------------------
        cut_region  = and(set, set_unsafe); % intersection region whose boundaries are cutting lines
        cut_lines   = cell(dim(set),1); % number of cut lines in each dim
        seg_nums    = zeros(dim(set),1); % numbers of segments in each dim
        for i = 1:dim(set)
            cut_lines_dim_i     = set.inf(i); % cut lines in i'th dimension
            if cut_region.inf(i) > set.inf(i) && cut_region.inf(i) < set.sup(i)
                cut_lines_dim_i = [cut_lines_dim_i, cut_region.inf(i)];
            end
            if cut_region.sup(i) > set.inf(i) && cut_region.sup(i) < set.sup(i)
                cut_lines_dim_i = [cut_lines_dim_i, cut_region.sup(i)];
            end
            cut_lines_dim_i = [cut_lines_dim_i, set.sup(i)];
            cut_lines{i}    = cut_lines_dim_i;
            seg_nums(i)     = length(cut_lines_dim_i)-1;
        end
        % -----------------------------------------------------------------
        % Cut the set into several elementary boxes
        % -----------------------------------------------------------------
        trimedSet   = {};
        for i_th_box = 1:prod(seg_nums)
            box_inf = NaN(dim(set),1);
            box_sup = NaN(dim(set),1);
            % Form the i'th segment/box
            indexs  = recoverIdx(seg_nums, i_th_box);
            for dim_i = 1:dim(set)
                box_inf(dim_i)  = cut_lines{dim_i}(indexs(dim_i));
                box_sup(dim_i)  = cut_lines{dim_i}(indexs(dim_i)+1);
            end
            % Keep the safe one
            box     = interval(box_inf, box_sup);
            if ~in(set_unsafe, box)
                trimedSet{end+1}    = box;
            end
        end
    end
end

function indexs = recoverIdx(dims, number)
    % Function used to recover the index fron number
    % Input: 
    %   dims: dimensions of the for-loop e.g. [I,J,K]
    %   number: idx number in the for-loop e.g. number =
    %       k+(j-1)*K+(i-1)*J*K
    % Output:
    %   indexs: [i,j,k]
    dim_num     = size(dims(:),1);
    indexs      = zeros(1, dim_num);
    residual    = number - 1;
    for dim_i = 1:dim_num-1        
        indexs(dim_i)   = floor(residual / prod(dims(dim_i+1:end))) + 1;
        residual        = mod(residual, prod(dims(dim_i+1:end)));      
    end
    indexs(end) = residual + 1;
end