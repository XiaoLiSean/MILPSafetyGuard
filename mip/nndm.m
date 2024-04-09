classdef nndm < matlab.mixin.Copyable
    % Neural Network Dynamic Model
    % By Xiao Li 
    properties
        ell; % layer number of the NN
        Weights; % weights matrices of NN, Ws{i} for i'th layer
        biases; % bias vectors of NN, Ws{i} for i'th layer
        n_x; % dimension of the state space
        n_u; % dimension of the control space
        n_is; % n_is{i} number of neurons in i'th hidden layer
        S_ws; % a cell of matrices Swijs{i}{j} = S(wij) used for calculating interval propagation
    end
    
    methods
        % =================================================================
        % Initialize the NNDM using toy parameters
        % =================================================================
        function obj = nndm(dynamicSys)
            % -------------------------------------------------------------
            % Initialize nndm structural parameters and these of states, controls
            % -------------------------------------------------------------
            obj.ell     = length(dynamicSys.NN.Weights);
            obj.Weights = dynamicSys.NN.Weights;
            obj.biases  = dynamicSys.NN.biases;
            obj.n_x         = dynamicSys.n_x;
            obj.n_u         = dynamicSys.n_u;
            % -------------------------------------------------------------
            % Preliminary verification of nndm parameters
            % -------------------------------------------------------------
            for i = 1:obj.ell
                obj.n_is{i}     = length(obj.biases{i});
                if size(obj.Weights{i},1) ~= length(obj.biases{i})
                    error(strcat('Dim of weight and bias miss-match in layer ', num2str(i)));
                end
            end
            for i = 2:obj.ell
                if size(obj.Weights{i},2) ~= size(obj.Weights{i-1},1)
                    error(strcat('Dim of weights miss-match in previous adjacent layer ', num2str(i)));
                end
            end            
            if size(obj.Weights{1},2) ~= (obj.n_x+obj.n_u)
                error('Dim of nn input miss-match');
            end
            if size(obj.Weights{obj.ell},1) ~= (obj.n_x)
                error('Dim of nn output miss-match');
            end
            % -------------------------------------------------------------
            % Functions used to initialize the matrices that are used to
            % calculate neural value bounds
            % -------------------------------------------------------------
            obj.initializeSws();
        end
        
        % =================================================================
        % Forward propagate the input [xk; uk] through the nndm
        % =================================================================
        function xkp1 = propagate(obj, xk, uk)
            z       = [xk; uk];
            for i = 1:obj.ell-1
                % Fully connected nndm with ReLU activation function
                z_hat   = obj.Weights{i}*z + obj.biases{i};
                z       = z_hat;
                z(z<0)  = 0;
            end
            xkp1    = obj.Weights{obj.ell}*z + obj.biases{obj.ell};
        end
        
        % =================================================================
        % Reachable set of input sets [Xk; Uk] through the nndm
        % =================================================================
        function Xkp1 = getReachableSet(obj, Xk, Uk)
            [z_hat_u, z_hat_l]  = obj.getNeuralValueBounds(Xk, Uk);
            Xkp1    = interval(z_hat_l{obj.ell}, z_hat_u{obj.ell});
        end
        
        % =================================================================
        % Initialize the matrices Swijs{i}{j} = S(wij): detialed info refer
        % to page 3 in the PPT (used to switch the upper and lower bounds
        % according to the sign of the weights for interval arithmetics
        % propagated through the nndm)
        % =================================================================
        function initializeSws(obj)
            for i = 1:obj.ell
                for j = 1:size(obj.Weights{i}, 1)
                    n_i_1           = size(obj.Weights{i}, 2);
                    obj.S_ws{i}{j}  = zeros(n_i_1, 2*n_i_1);
                    wij             = obj.Weights{i}(j,:);
                    for q = 1:n_i_1
                        sq  = zeros(2*n_i_1,1);
                        if wij(q) >= 0
                            sq(q)       = 1;
                        else
                            sq(q+n_i_1) = 1;
                        end
                        obj.S_ws{i}{j}(q,:)   = sq';
                    end
                end
            end
        end
        
        % =================================================================
        % Calculate the bounds of values in each neuron using feasible sets
        % and the initialized S_wijs{i}{j} = S(wij)
        % Inputs:
        %   X: feasible set of the states
        %   U: feasible set of the control
        % Outputs:
        %   z_hat_u: z_hat_u{i}(j) is the upper bound of the pre-activated value in i'th layer j'th neuron calculated from U and X 
        %   z_hat_l: z_hat_l{i}(j) is the lower bound of ...
        % =================================================================
        function [z_hat_u, z_hat_l] = getNeuralValueBounds(obj, X, U)
            z_l     = [X.inf; U.inf];
            z_u     = [X.sup; U.sup];
            z_hat_l = cell(obj.ell, 1);
            z_hat_u = cell(obj.ell, 1);
            for i = 1:obj.ell
                z_hat_l{i}  = zeros(length(obj.biases{i}), 1);
                z_hat_u{i}  = zeros(length(obj.biases{i}), 1);
                for j = 1:size(obj.Weights{i}, 1)
                    z_hat_l{i}(j)   = dot(obj.Weights{i}(j,:), obj.S_ws{i}{j}*[z_l; z_u]) + obj.biases{i}(j);
                    z_hat_u{i}(j)   = dot(obj.Weights{i}(j,:), obj.S_ws{i}{j}*[z_u; z_l]) + obj.biases{i}(j);
                end
                z_l         = z_hat_l{i};
                z_l(z_l<0)  = 0;
                z_u         = z_hat_u{i};
                z_u(z_u<0)  = 0;
            end
            obj.verifyNNBounds(X, U, z_hat_u, z_hat_l)
        end
        
        % =================================================================
        % Function used to verify the interval  procedure by
        % Fcns "initializeSws" and "initializeNNBounds" (based on CORA)
        % =================================================================
        function verifyNNBounds(obj, X, U, z_hat_u, z_hat_l)
            Z   = interval([X.inf; U.inf], [X.sup; U.sup]);
            for i = 1:obj.ell-1
                % Fully connected nndm with ReLU activation function
                Z_hat   = obj.Weights{i}*Z + obj.biases{i};
                if ~obj.numericallyEq(Z_hat.inf, z_hat_l{i}) || ~obj.numericallyEq(Z_hat.sup, z_hat_u{i})
                    warning(strcat('Interval arithmetics test fail in layer ', num2str(i)));
                end
                Z_inf   = Z_hat.inf;
                Z_sup   = Z_hat.sup;
                for dim_i = 1:dim(Z_hat)
                    if Z_hat.inf(dim_i) <= 0
                        Z_inf(dim_i)    = 0;
                    end
                    if Z_hat.sup(dim_i) <= 0
                        Z_sup(dim_i)    = 0;
                    end
                end
                Z       = interval(Z_inf, Z_sup);
            end
            xkp1_tild   = obj.Weights{obj.ell}*Z + obj.biases{obj.ell};
            if ~obj.numericallyEq(xkp1_tild.inf, z_hat_l{obj.ell}) || ~obj.numericallyEq(xkp1_tild.sup, z_hat_u{obj.ell})
                warning('Interval arithmetics test fail in final layer');
            end
        end
        
        % Function used to test if two numbers are equal despite a certain
        % level of insignificant numerical differences
        function isEqual = numericallyEq(obj, x, y)
            rel     = abs(x-y) < 1e4*eps(min(abs(x),abs(y)));
            isEqual = isequal(rel, true(size(rel)));
        end
    end
end