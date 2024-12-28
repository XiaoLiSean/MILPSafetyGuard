classdef ominidirectionalRobot < matlab.mixin.Copyable
    % Simple Dynamic Model
    % By Xiao Li 
    properties
        n_x; % dimension of the state space
        n_u; % dimension of the control space
        
        X; % feasible set of the states
        X_us; % X_us{i} is the i'th unsafe box
        U; % feasible set of the control
        
        xk; % actual current state
        xk_tild; % current state measurement
        
        epsilon_u; % error bound of the actuator noise
        epsilon_x; % error bound of the model prediction
        epsilon_y; % error bound of the sensor noise
        
        NN; % pre-trained nn weights and bias
    end
    
    methods
        % =================================================================
        % Initialize the toy dynamic system
        % =================================================================
        function obj = ominidirectionalRobot(x0)
            % -------------------------------------------------------------
            % Initialize constrains and bounds on dynamic system
            % -------------------------------------------------------------
            obj.X       = interval([-10; -10], [10; 10]);
            obj.U       = interval([-0.25; -0.25], [0.25; 0.25]);
            obj.X_us    = {};
            obj.X_us{1} = interval([7; 5], [8; 10.01]);
            obj.X_us{2} = interval([5; 3], [6; 6]);
            obj.X_us{3} = interval([2; 1], [8; 2]);
            obj.X_us{4} = interval([0; 7], [6; 8]);
            obj.X_us{5} = interval([2; -1], [6; 0]);
            obj.X_us{6} = interval([7; 3], [8; 4]);
            obj.X_us{7} = interval([3; 2], [4; 3]);
            obj.X_us{8} = interval([0; 3], [2; 4]);
            obj.X_us{9} = interval([3; 4], [4; 5]);
            obj.X_us{10}= interval([1; 5], [3; 6]);
            obj.X_us{11}= interval([7; 0], [8; 1]);
            obj.X_us{12}= interval([0; 1], [1; 2]);
            obj.X_us{13}= interval([8; 5], [9; 6]);
            obj.X_us{14}= interval([0; 8], [1; 10.01]);
            obj.n_x     = dim(obj.X);
            obj.n_u     = dim(obj.U);
            obj.epsilon_u   = 0.05*ones(obj.n_u, 1);
            obj.epsilon_x   = 0.05*ones(obj.n_x, 1);
            obj.epsilon_y   = 0.05*ones(obj.n_x, 1);
            obj.initializeDynamicSystem(x0)
            
            % Define simple system
            obj.NN.Weights{1}   = diag([-1,-1,-1,-1]);
            obj.NN.Weights{2}   = [-eye(2), -eye(2)];
            obj.NN.biases{1}    = [50;50;50;50];
            obj.NN.biases{2}    = [100;100];    
        end
        % =================================================================
        % Initialize the state and measurement with x0
        % =================================================================  
        function initializeDynamicSystem(obj, x0)
            if in(obj.X, x0) == 0
                error('initial state is outside the feasisble set');
            else
                for i = 1:length(obj.X_us)
                    if in(obj.X_us{i}, x0) == 1
                        error('initial state is inside the unsafe sets');
                    end
                end
            end
            obj.xk          = x0;
            obj.xk_tild     = obj.getMeasurement();
        end
        
        % =================================================================
        % Update the state and measurement with computed control uk_tild
        % =================================================================
        function updateDynamicSystem(obj, uk_tild)
            uk          = uk_tild + diag((2*rand([obj.n_u,1])-1))*obj.epsilon_u;
            obj.xk      = obj.updateState(uk);
            obj.xk_tild = obj.getMeasurement();
        end   
        
        % =================================================================
        % State Dynamics
        % =================================================================
        function xkp1 = updateState(obj, uk)
            % Threshold control signal
            uk          = min(max(uk, obj.U.inf), obj.U.sup);
            z           = [obj.xk; uk];
            for i = 1:1
                % Fully connected nndm with ReLU activation function
                z_hat   = obj.NN.Weights{i}*z + obj.NN.biases{i};
                z       = z_hat;
                z(z<0)  = 0;
            end
            xkp1    = obj.NN.Weights{2}*z + obj.NN.biases{2};
            xkp1    = xkp1 + diag((2*rand([obj.n_x,1])-1))*obj.epsilon_x;
        end
        % =================================================================
        % Measurement Model
        % =================================================================
        function yk = getMeasurement(obj)
            yk  = obj.xk + diag((2*rand([obj.n_x,1])-1))*obj.epsilon_y;
        end
    end
end