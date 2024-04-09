classdef reachabilityGuidedRRT < matlab.mixin.Copyable
    % Reachability Guided RRT
    % By Xiao Li 
    properties
        source; % start point
        target; % goal point
        sys; % dynamic system
        nn; % neural network
        
        nodes_num; % number of nodes
        nodes; % nodes{i} contains the coordinates and reachable set of i'th node
        edges; % contains the start, end nodes and the distance metric as arrays
        directedGraph; % directed graph tree
        shortestPath; % shortest path from source to target
        shortestRefTrajectory; % shortest reference trajectory from source to target
    end
    
    methods
        % =================================================================
        % Initialize the rrt
        % =================================================================
        function obj = reachabilityGuidedRRT(source, target, dynamicSys, neuralNetwork)
            obj.source  = source;
            obj.target  = target;
            obj.sys     = dynamicSys;
            obj.nn      = neuralNetwork;
            
            obj.nodes_num           = 0;
            obj.nodes.coordinates   = [];
            obj.nodes.reachableSet  = {};
            obj.addNode(obj.source);
            obj.edges.startNodes    = [];
            obj.edges.endNodes      = [];
            obj.edges.distances     = [];
        end
        
       
        % =================================================================
        % Main RRT to explore the state space
        % =================================================================
        function exploreStateSpace(obj, is_visualization)
            isReached   = false;
            while ~isReached   
                % ---------------------------------------------------------
                % Draw random sample from the state space
                % ---------------------------------------------------------
                pt_sample   = obj.samplePt();
                
                % ---------------------------------------------------------
                % Perform Reachability Guided RRT
                % ---------------------------------------------------------
                [ith_node, is_RSet, pt_RSet]    = obj.findNearestL1(pt_sample);
                if is_RSet && obj.addNode(pt_RSet)
                    start_idx   = ith_node;
                    end_idx     = obj.nodes_num;
                    start_pt    = obj.nodes.coordinates(:, ith_node);
                    end_pt      = pt_RSet;
                    obj.addEdgeL1(start_idx, end_idx, start_pt, end_pt);
                end

                % ---------------------------------------------------------
                % Check if the target is in one of the reachable set
                % ---------------------------------------------------------
                [isReached, ith_RSet]   = obj.reachedTheTarget();
                if isReached
                    obj.addNode(obj.target);
                    start_idx   = ith_RSet;
                    end_idx     = obj.nodes_num;
                    start_pt    = obj.nodes.coordinates(:, ith_RSet);
                    end_pt      = obj.target;
                    obj.addEdgeL1(start_idx, end_idx, start_pt, end_pt);
                end
                obj.directedGraph   = digraph(obj.edges.startNodes, obj.edges.endNodes, obj.edges.distances);
                
                % ---------------------------------------------------------
                % Visualize the tree growth
                % ---------------------------------------------------------
                if is_visualization
                    if exist('hGraph', 'var')
                        delete(hGraph);
                        delete(hSample);
                    end
                    
                    hGraph  = plot(obj.directedGraph, 'EdgeColor', [0.5 0.5 0.5], 'NodeColor', [0.5 0.5 0.5], 'LineWidth', 5,...
                        'NodeLabel', {}, 'XData', obj.nodes.coordinates(1,:), 'YData', obj.nodes.coordinates(2,:)); hold on;
                    hSample = plot(pt_sample(1), pt_sample(2), 'r.', 'markerSize', 20, 'lineWidth', 5); hold on;
                    hNode   = plot(obj.nodes.coordinates(1,:), obj.nodes.coordinates(2,:), '.', 'color', [0.8 0.8 0.8], 'markerSize', 20);
                    pause(0.02)
                end
            end
            
            % ---------------------------------------------------------
            % Find the shortest path
            % ---------------------------------------------------------
            [obj.shortestPath, ~]       = shortestpath(obj.directedGraph, 1, obj.nodes_num);
            obj.shortestRefTrajectory   = obj.nodes.coordinates(:, obj.shortestPath);
            
            % ---------------------------------------------------------
            % Visualize the shortest path
            % ---------------------------------------------------------
            if is_visualization
                if exist('hGraph', 'var')
                    delete(hGraph);
                    delete(hSample);
                    delete(hNode);
                end
                hGraph  = plot(obj.directedGraph, 'EdgeColor', [0.8 0.8 0.8], 'NodeColor', [0.8 0.8 0.8], 'LineWidth', 5,...
                        'NodeLabel', {}, 'XData', obj.nodes.coordinates(1,:), 'YData', obj.nodes.coordinates(2,:)); hold on;
                plot(obj.nodes.coordinates(1,:), obj.nodes.coordinates(2,:), '.', 'color', [0.8 0.8 0.8], 'markerSize', 20);
                highlight(hGraph, obj.shortestPath, 'NodeColor', [154,43,43]./225, 'EdgeColor', 'k', 'LineWidth', 5);
                plot(obj.nodes.coordinates(1, obj.shortestPath), obj.nodes.coordinates(2, obj.shortestPath), '.', 'color', [154,43,43]./225, 'markerSize', 20);
            end
        end
        
        % =================================================================
        % Check the termination condition of the RRT exploration
        % =================================================================
        function [isReached, ith_RSet] = reachedTheTarget(obj)
            isReached   = false;
            ith_RSet    = 0;
            for i = 1:obj.nodes_num
                for j = 1:length(obj.nodes.reachableSet{i})
                    if in(obj.nodes.reachableSet{i}{j}, obj.target)
                        isReached   = true;
                        ith_RSet    = i;
                        return
                    end
                end
            end
        end
        
        % =================================================================
        % Add node and its reachable set to the cell variable nodes
        % =================================================================
        function is_marchable = addNode(obj, coordinates)
            Xk          = interval(coordinates, coordinates);
            Uk          = obj.sys.U;
            Xkp1        = obj.nn.getReachableSet(Xk, Uk);
            trimedXkpq  = trimUnsafeRegions(Xkp1, obj.sys.X_us);
            if isempty(trimedXkpq)
                is_marchable    = false;
            else
                obj.nodes.coordinates(:, end+1) = coordinates;
                obj.nodes.reachableSet{end+1}   = trimedXkpq;
                obj.nodes_num   = obj.nodes_num + 1;
                is_marchable    = true;
            end
        end
        
        % =================================================================
        % Add edge between existing nodes
        % =================================================================
        function addEdgeL1(obj, start_idx, end_idx, start_pt, end_pt)
            distance    = norm(start_pt-end_pt, 1);
            obj.edges.startNodes(end+1) = start_idx;
            obj.edges.endNodes(end+1)   = end_idx;
            obj.edges.distances(end+1)  = distance;
        end    
        
        % =================================================================
        % Find the closest node or reachable set point to the sample
        % use the L1 norm as distance metric
        % =================================================================
        function [ith_node, is_RSet, pt_RSet] = findNearestL1(obj, pt_sample)
            to_node_distance    = zeros(obj.nodes_num, 1);
            to_RSet_distance    = zeros(obj.nodes_num, 1);
            closest_pt_in_RSet  = zeros(obj.sys.n_x, obj.nodes_num);
            for i = 1:obj.nodes_num
                to_node_distance(i)     = norm(obj.nodes.coordinates(:,i)-pt_sample,1);
                [pt, distance]          = findNearestPtInSetsL1(obj.nodes.reachableSet{i}, pt_sample);
                to_RSet_distance(i)     = distance;
                closest_pt_in_RSet(:,i) = pt; 
            end
            
            if min(to_node_distance) < min(to_RSet_distance) 
                is_RSet     = false;
                ith_node    = -1;
                pt_RSet     = [];
            else
                is_RSet         = true;
                [~, ith_node]   = min(to_RSet_distance);
                pt_RSet         = closest_pt_in_RSet(:, ith_node);
            end
        end
              
        % =================================================================
        % Uniformly draw sample from the state space exclude the obstacles
        % =================================================================
        function pt_sample = samplePt(obj)
            is_unsafe   = true;
            while is_unsafe
                try 
                    pt_sample   = sampleBox(zonotope(obj.sys.X), 1);
                catch
                    Z           = zonotope(obj.sys.X);
                    pt_sample   = Z.randPoint(1, 'uniform');
                end
                is_unsafe   = false;
                for i = 1:length(obj.sys.X_us)
                    if in(obj.sys.X_us{i}, pt_sample)
                        is_unsafe   = true;
                        break
                    end
                end
            end
        end
    end
end