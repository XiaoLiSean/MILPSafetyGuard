classdef safeOptCtrl < matlab.mixin.Copyable
    % MIP embedding of NNDM using YALMIP 
    % By Xiao Li 
    properties
        NN; % neural network dynamics model
        SYS; % dynamic system
        Constraints; % optimization constraints
        Objective; % optimization objective
        % -----------------------------------------------------------------
        % Optimization variable (sdpvar) for intervals in neurons
        % -----------------------------------------------------------------
        a_0; % lower bounds of input layer
        a_hat_is; % a_hat_is{i} (i=1,...,l-1) lower bounds of pre-activated hidden layers
        a_is; % a_is{i} (i=1,...,l-1) lower bounds of activated hidden layers
        a_kp1; % lower bounds of output layer
        b_0; % upper bounds of input layer
        b_hat_is; % b_hat_is{i} (i=1,...,l-1) upper bounds of pre-activated hidden layers
        b_is; % b_is{i} (i=1,...,l-1) upper bounds of activated hidden layers
        b_kp1; % upper bounds of output layer
        x_kp1_l; % lower bound of x_{k+1}
        x_kp1_u; % upper bound of x_{k+1}        
        % -----------------------------------------------------------------
        % Optimization variable (binvar) for input constraints
        % -----------------------------------------------------------------
        delta_a; % delta_a{i} == 1 means U.inf(i) >= u_tild(i) - epsilon_u
        delta_b; % delta_b{i} == 1 means U.sup(i) <= u_tild(i) + epsilon_u        
        % -----------------------------------------------------------------
        % Optimization variable (binvar) for output safety constraints
        % (obstacle avoidance)
        % -----------------------------------------------------------------
        delta_u_1s; % delta_u_1s{i}(j) == 1 means x_kp1_u(j) <= NN.X_us{i}(j).inf
        delta_u_2s; % delta_u_2s{i}(j) == 1 means x_kp1_l(j) >= NN.X_us{i}(j).sup
        % -----------------------------------------------------------------
        % Optimization variable (binvar) for activation status in neurons
        % -----------------------------------------------------------------
        delta_mms; % delta_mms{i}(j) == 1 means a_hat_is{i}(j) <= b_hat_is{i}(j) <= 0
        delta_mps; % delta_mps{i}(j) == 1 means a_hat_is{i}(j) <= 0 <= b_hat_is{i}(j)
        delta_pps; % delta_pps{i}(j) == 1 means 0 <= a_hat_is{i}(j) <= b_hat_is{i}(j)
        % -----------------------------------------------------------------
        % Optimization initial conditions, constrains and tracking signals
        % -----------------------------------------------------------------
        x_r; % reference state 
        xk_tild; % current state measurement                
        z_hat_u; % z_hat_u{i}(j) is the upper bound of the pre-activated value in i'th layer j'th neuron calculated from U and X 
        z_hat_l; % z_hat_l{i}(j) is the lower bound of ...
        % -----------------------------------------------------------------
        % Optimization objective
        % -----------------------------------------------------------------
        u_tild; % optimal computed control signal
        lambda; % slack variable for linearization of L1 norm objective
    end
    methods
        % =================================================================
        % Initialize the safe optimal control problem
        % =================================================================
        function obj = safeOptCtrl(x_r, dynamicSys, neuralNetwork)
            yalmip('clear');
            % -------------------------------------------------------------
            % Initialize NN and state variables and measurements
            % -------------------------------------------------------------            
            obj.NN      = neuralNetwork;
            obj.SYS     = dynamicSys;
            obj.u_tild  = sdpvar(obj.SYS.n_u, 1); % optimal ctrl variable
            obj.setCurrentStateMeasurement(obj.SYS.getMeasurement());
            obj.setCurrentNeuralValueBounds(); % set the bounds
            obj.setRefState(x_r);
            % -------------------------------------------------------------
            % Initialize constraints
            % -------------------------------------------------------------            
            obj.Constraints     = [];
            obj.initializeInputConstraints();
            obj.initializeHiddenLayerConstraints();
            obj.initializeOutputConstraints();
            if isa(dynamicSys, 'vehicleBicycleModel')
                obj.initializeObjectiveVehicle();
            elseif isa(dynamicSys, 'ominidirectionalRobot')
                obj.initializeObjectiveL1();
            else
                error('This class of dynamics is not integrated');
            end
        end
        
        function result = solveSafeOptCtrlProblem(obj)
            result  = optimize(obj.Constraints, obj.Objective);
        end
        
        % =================================================================
        % Function used to intialize constraints for input feasibility
        % guarantee w.r.t. to safety and model assumption (PPT page 2)
        % =================================================================
        function initializeInputConstraints(obj)
            obj.a_0         = sdpvar(obj.SYS.n_x+obj.SYS.n_u, 1);
            obj.b_0         = sdpvar(obj.SYS.n_x+obj.SYS.n_u, 1);
            obj.Constraints = [obj.Constraints, obj.a_0 <= obj.b_0];
            % -------------------------------------------------------------
            % Constraints w.r.t. state feasible set and measurement
            % -------------------------------------------------------------
            obj.Constraints     = [obj.Constraints, obj.a_0(1:obj.SYS.n_x) == max(obj.SYS.X.inf, obj.xk_tild-obj.SYS.epsilon_y)];
            obj.Constraints     = [obj.Constraints, obj.b_0(1:obj.SYS.n_x) == min(obj.SYS.X.sup, obj.xk_tild+obj.SYS.epsilon_y)];
            % -------------------------------------------------------------
            % Constraints w.r.t. non-empty intersection of U and u_tild set
            % -------------------------------------------------------------
            obj.Constraints     = [obj.Constraints, obj.u_tild >= obj.SYS.U.inf];
            obj.Constraints     = [obj.Constraints, obj.u_tild <= obj.SYS.U.sup];            
            % -------------------------------------------------------------
            % Constraints w.r.t. control feasible set and u_tild
            % -------------------------------------------------------------
            % m   = max(2*obj.SYS.epsilon_u, obj.SYS.U.sup - obj.SYS.U.inf);
            m   = max(obj.SYS.epsilon_u, obj.SYS.U.sup - obj.SYS.U.inf - obj.SYS.epsilon_u);
            obj.delta_a         = binvar(obj.SYS.n_u, 1);
            obj.delta_b         = binvar(obj.SYS.n_u, 1);
            % constraint lower bounds
            obj.Constraints     = [obj.Constraints, obj.a_0((obj.SYS.n_x+1):end) >= obj.SYS.U.inf];
            obj.Constraints     = [obj.Constraints, obj.a_0((obj.SYS.n_x+1):end) >= (obj.u_tild - obj.SYS.epsilon_u)];
            obj.Constraints     = [obj.Constraints, obj.a_0((obj.SYS.n_x+1):end) <= (obj.SYS.U.inf + diag(m)*(1-obj.delta_a))];
            obj.Constraints     = [obj.Constraints, obj.a_0((obj.SYS.n_x+1):end) <= (obj.u_tild - obj.SYS.epsilon_u + diag(m)*obj.delta_a)];
            % constraint upper bounds
            obj.Constraints     = [obj.Constraints, obj.b_0((obj.SYS.n_x+1):end) <= obj.SYS.U.sup];
            obj.Constraints     = [obj.Constraints, obj.b_0((obj.SYS.n_x+1):end) <= (obj.u_tild + obj.SYS.epsilon_u)];
            obj.Constraints     = [obj.Constraints, obj.b_0((obj.SYS.n_x+1):end) >= (obj.SYS.U.sup - diag(m)*(1-obj.delta_b))];
            obj.Constraints     = [obj.Constraints, obj.b_0((obj.SYS.n_x+1):end) >= (obj.u_tild + obj.SYS.epsilon_u - diag(m)*obj.delta_b)];
        end
        
        % =================================================================
        % Function used to intialize constraints to embed NNDM fully
        % connected layers (i=1,...,l-1) and the ReLU activate functions
        % (refer to PPT page (3-6)
        % =================================================================
        function initializeHiddenLayerConstraints(obj)
            for i = 1:(obj.NN.ell-1)
                obj.initialize_ith_Fcn_Constraints(i);
                obj.initialize_ith_ReLU_Constraints(i);
            end
        end
        
        % =================================================================
        % Function used to intialize constraints to embed NNDM fully
        % connected i'th layer (i=1,...,l-1) (refer to PPT page (3-4)
        % =================================================================       
        function initialize_ith_Fcn_Constraints(obj, i)
            obj.a_hat_is{i} = sdpvar(obj.NN.n_is{i}, 1);
            obj.a_is{i}     = sdpvar(obj.NN.n_is{i}, 1);
            obj.b_hat_is{i} = sdpvar(obj.NN.n_is{i}, 1);
            obj.b_is{i}     = sdpvar(obj.NN.n_is{i}, 1);
            obj.Constraints = [obj.Constraints, obj.a_hat_is{i} <= obj.b_hat_is{i}];
            obj.Constraints = [obj.Constraints, obj.a_hat_is{i} >= obj.z_hat_l{i}];
            obj.Constraints = [obj.Constraints, obj.b_hat_is{i} <= obj.z_hat_u{i}];
            for j = 1:obj.NN.n_is{i}
                if i == 1
                    obj.Constraints     = [obj.Constraints,...
                        obj.a_hat_is{i}(j) == (dot(obj.NN.Weights{i}(j,:), obj.NN.S_ws{i}{j}*[obj.a_0; obj.b_0]) + obj.NN.biases{i}(j))];
                    obj.Constraints     = [obj.Constraints,...
                        obj.b_hat_is{i}(j) == (dot(obj.NN.Weights{i}(j,:), obj.NN.S_ws{i}{j}*[obj.b_0; obj.a_0]) + obj.NN.biases{i}(j))];
                else
                    obj.Constraints     = [obj.Constraints,...
                        obj.a_hat_is{i}(j) == (dot(obj.NN.Weights{i}(j,:), obj.NN.S_ws{i}{j}*[obj.a_is{i-1}; obj.b_is{i-1}]) + obj.NN.biases{i}(j))];
                    obj.Constraints     = [obj.Constraints,...
                        obj.b_hat_is{i}(j) == (dot(obj.NN.Weights{i}(j,:), obj.NN.S_ws{i}{j}*[obj.b_is{i-1}; obj.a_is{i-1}]) + obj.NN.biases{i}(j))];
                end
            end
        end
       
        % =================================================================
        % Function used to intialize constraints to embed ReLU activation
        % functions for i'th layer (i=1,...,l-1) (refer to PPT page (5-6)
        % =================================================================       
        function initialize_ith_ReLU_Constraints(obj, i)
            % -------------------------------------------------------------
            % Constraints defined by triplet switching condition of the
            % ReLU activation function
            % -------------------------------------------------------------
            obj.delta_mms{i}    = binvar(obj.NN.n_is{i}, 1);
            obj.delta_mps{i}    = binvar(obj.NN.n_is{i}, 1);
            obj.delta_pps{i}    = binvar(obj.NN.n_is{i}, 1);
            obj.Constraints     = [obj.Constraints,...
                (obj.delta_mms{i}+obj.delta_mps{i}+obj.delta_pps{i}) == ones(obj.NN.n_is{i}, 1)];
            % -------------------------------------------------------------
            % Constraints w.r.t. non-negative property of ReLU
            % -------------------------------------------------------------
            obj.Constraints     = [obj.Constraints, obj.b_is{i} >= obj.a_is{i}];
            obj.Constraints     = [obj.Constraints, obj.a_is{i} >= zeros(obj.NN.n_is{i}, 1)];            
            % -------------------------------------------------------------
            % Constraints w.r.t. lower bounds through ReLU
            % -------------------------------------------------------------
            obj.Constraints     = [obj.Constraints, obj.a_is{i} >= obj.a_hat_is{i}];
            obj.Constraints     = [obj.Constraints, obj.a_is{i} <= (obj.a_hat_is{i} - diag(obj.z_hat_l{i})*(obj.delta_mms{i}+obj.delta_mps{i}))];
            obj.Constraints     = [obj.Constraints, obj.a_is{i} <= (diag(obj.z_hat_u{i})*obj.delta_pps{i})];
            % -------------------------------------------------------------
            % Constraints w.r.t. upper bounds through ReLU
            % -------------------------------------------------------------    
            obj.Constraints     = [obj.Constraints, obj.b_is{i} >= obj.b_hat_is{i}];
            obj.Constraints     = [obj.Constraints, obj.b_is{i} <= (obj.b_hat_is{i} - diag(obj.z_hat_l{i})*obj.delta_mms{i})];
            obj.Constraints     = [obj.Constraints, obj.b_is{i} <= (diag(obj.z_hat_u{i})*(obj.delta_mps{i}+obj.delta_pps{i}))];                 
        end

        % =================================================================
        % Function used to intialize constraints for Output feasibility
        % guarantee w.r.t. to safety and model assumption (PPT page 7)
        % =================================================================
        function initializeOutputConstraints(obj)
            obj.a_kp1       = sdpvar(obj.SYS.n_x, 1);
            obj.b_kp1       = sdpvar(obj.SYS.n_x, 1);
            obj.x_kp1_l     = sdpvar(obj.SYS.n_x, 1);
            obj.x_kp1_u     = sdpvar(obj.SYS.n_x, 1);
            obj.Constraints = [obj.Constraints, obj.a_kp1 <= obj.b_kp1];
            obj.Constraints = [obj.Constraints, obj.x_kp1_l <= obj.x_kp1_u];
            % -------------------------------------------------------------
            % Constraints w.r.t. final fully connected layer
            % -------------------------------------------------------------
            i   = obj.NN.ell;
            for j = 1:obj.SYS.n_x
                obj.Constraints     = [obj.Constraints,...
                    obj.a_kp1(j) == (dot(obj.NN.Weights{i}(j,:), obj.NN.S_ws{i}{j}*[obj.a_is{i-1}; obj.b_is{i-1}]) + obj.NN.biases{i}(j))];
                obj.Constraints     = [obj.Constraints,...
                    obj.b_kp1(j) == (dot(obj.NN.Weights{i}(j,:), obj.NN.S_ws{i}{j}*[obj.b_is{i-1}; obj.a_is{i-1}]) + obj.NN.biases{i}(j))];
            end
            % -------------------------------------------------------------
            % Constraints w.r.t. state feasible set, pre-computed reachable
            % set, and bounded model imperfections 
            % -------------------------------------------------------------
            obj.Constraints     = [obj.Constraints, obj.x_kp1_l == (obj.a_kp1-obj.SYS.epsilon_x)];
            obj.Constraints     = [obj.Constraints, obj.x_kp1_u == (obj.b_kp1+obj.SYS.epsilon_x)];
            obj.Constraints     = [obj.Constraints, obj.a_kp1 >= obj.z_hat_l{i}];
            obj.Constraints     = [obj.Constraints, obj.b_kp1 <= obj.z_hat_u{i}];
            % -------------------------------------------------------------
            % Constraints w.r.t. obstacles
            % -------------------------------------------------------------
            if isempty(obj.SYS.X_us)
                obj.Constraints = [obj.Constraints, obj.x_kp1_l >= obj.SYS.X.inf];
                obj.Constraints = [obj.Constraints, obj.x_kp1_u <= obj.SYS.X.sup];
            else
                X_us    = feasibleObstables(obj);
                for i = 1:length(X_us)
                    % Initialize optimization variables
                    obj.delta_u_1s{i}   = binvar(obj.SYS.n_x, 1); 
                    obj.delta_u_2s{i}   = binvar(obj.SYS.n_x, 1);
                    obj.Constraints     = [obj.Constraints, obj.delta_u_1s{i} + obj.delta_u_2s{i} <= ones(obj.SYS.n_x,1)];
                    % Obstacle avoidance
                    obj.Constraints = [obj.Constraints, obj.x_kp1_u <= obj.SYS.X.sup + diag(obj.delta_u_1s{i})*(X_us{i}.inf-obj.SYS.X.sup)];
                    obj.Constraints = [obj.Constraints, obj.x_kp1_l >= obj.SYS.X.inf + diag(obj.delta_u_2s{i})*(X_us{i}.sup-obj.SYS.X.inf)];
                    obj.Constraints = [obj.Constraints, obj.x_kp1_u >= X_us{i}.inf - diag(obj.delta_u_1s{i})*(X_us{i}.inf-obj.SYS.X.inf)];
                    obj.Constraints = [obj.Constraints, obj.x_kp1_l <= X_us{i}.sup - diag(obj.delta_u_2s{i})*(X_us{i}.sup-obj.SYS.X.sup)];                  
                    obj.Constraints = [obj.Constraints, sum(obj.delta_u_1s{i}) + sum(obj.delta_u_2s{i}) >= 1];
                end
            end
        end
        
        % =================================================================
        % Function used to intialize optimization objective (PPT page 8)
        % =================================================================        
        function initializeObjectiveL1(obj)
            obj.lambda          = sdpvar(obj.SYS.n_x, 1);
            obj.Constraints     = [obj.Constraints, obj.lambda >= zeros(obj.SYS.n_x, 1)];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_r - obj.x_kp1_l)];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_kp1_l - obj.x_r)];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_r - obj.x_kp1_u)];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_kp1_u - obj.x_r)];
            obj.Objective       = sum(obj.lambda);
        end

        % =================================================================
        % Function used to intialize optimization objective (Vehicle Test)
        % =================================================================        
        function initializeObjectiveVehicle(obj)
            obj.lambda          = sdpvar(obj.SYS.n_x - 1, 1);
            obj.Constraints     = [obj.Constraints, obj.lambda >= zeros(obj.SYS.n_x - 1, 1)];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_r(1:2) - obj.x_kp1_l(1:2))];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_kp1_l(1:2) - obj.x_r(1:2))];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_r(1:2) - obj.x_kp1_u(1:2))];
            obj.Constraints     = [obj.Constraints, obj.lambda >= (obj.x_kp1_u(1:2) - obj.x_r(1:2))];
            obj.Objective       = sum(obj.lambda);
        end

        % =================================================================
        % Use the reachable set to eliminate obstable constraints
        % =================================================================
        function X_us = feasibleObstables(obj)
            reachableSet    = interval(obj.z_hat_l{end}-obj.SYS.epsilon_x, obj.z_hat_u{end}+obj.SYS.epsilon_x);
            X_us            = {};
            for i = 1:length(obj.SYS.X_us)
                if ~isempty(and(reachableSet, obj.SYS.X_us{i}))
                    X_us{end+1}     = obj.SYS.X_us{i};
                end
            end
        end
        
        % =================================================================
        % Use the state measurement and control feasible set to calculate 
        % tighter bounds for neural values
        % =================================================================
        function setCurrentNeuralValueBounds(obj)
            Xk  = interval(obj.xk_tild-obj.SYS.epsilon_y, obj.xk_tild+obj.SYS.epsilon_y);
            Xk  = and(obj.SYS.X, Xk);
            [obj.z_hat_u, obj.z_hat_l]  = obj.NN.getNeuralValueBounds(Xk, obj.SYS.U); 
        end  
        
        % Set the current state measurement
        function setCurrentStateMeasurement(obj, xk_tild)
            obj.xk_tild         = xk_tild;
        end  
        
        % Set the current reference state
        function setRefState(obj, x_r)
            obj.x_r     = x_r;
        end
              
    end
end