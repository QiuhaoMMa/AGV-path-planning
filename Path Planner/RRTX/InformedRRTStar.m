classdef InformedRRTStar < RRTS
    properties
        cMax = inf; % Initial c_max (used in informed sampling)
        Xsoln = []; % Set of solution nodes
        goalThreshold = 0.5; % Threshold distance to define if node is in goal region
        xGoal; % Goal position
    end

    methods (Access = public)
        % Constructor to initialize InformedRRTStar with environment and goal position
        function obj = InformedRRTStar(environment, goal, varargin)
            obj@RRTS(environment, varargin{:});
            obj.xGoal = goal;  % Set the goal position
        end
    end

    methods (Access = protected)
        % Overriding the sample method to use informed sampling
        function xRand = sample(obj)
            if obj.cMax < inf
                xRand = obj.informedSample(obj.cMax);
            else
                % Regular random sampling if no solution exists
                xRand = obj.randomState();
            end
        end

        % Informed sampling method based on the algorithm provided
        function xRand = informedSample(obj, cMax)
            cMin = norm(obj.xGoal - obj.xStart);
            xCenter = (obj.xStart + obj.xGoal) / 2;
            C = obj.rotationToWorldFrame(obj.xStart, obj.xGoal);
            r1 = cMax / 2;
            r = [r1; sqrt(r1^2 - cMin^2) / 2]; % Radius for ellipsoid

            % Generate a random point in a unit n-ball
            xBall = obj.sampleUnitNBall();
            
            % Map the sample to the ellipsoid in the environment
            xRand = C * diag(r) * xBall + xCenter;

            % Ensure the sample is within bounds
            if ~obj.isWithinBounds(xRand)
                xRand = obj.randomState(); % Fallback to random sampling if outside bounds
            end
        end

        % Rotational matrix to transform the unit ball to the ellipsoid frame
        function C = rotationToWorldFrame(~, xStart, xGoal)
            % Assuming 2D space for simplicity
            d = xGoal - xStart;
            theta = atan2(d(2), d(1)); % Angle of the line connecting start and goal
            C = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end

        % Sample random point in a unit n-ball
        function xBall = sampleUnitNBall(obj)
            n = length(obj.xStart); % Dimensionality
            xBall = randn(n, 1); % Random Gaussian vector
            xBall = xBall / norm(xBall) * rand^(1/n); % Normalize and scale to unit ball
        end

        % Add a new node and update the solution set Xsoln if in goal region
        function addNode(obj, node, draw)
            % Call the base class addNode method
            addNode@RRTS(obj, node, draw);

            % Update cMax if the node is in the goal region
            if obj.inGoalRegion(node.position)
                obj.Xsoln = [obj.Xsoln, node];
                obj.cMax = min([obj.cMax, node.distance2start]);
            end
        end

        % Method to check if a node is within the goal region
        function inGoal = inGoalRegion(obj, position)
            % Calculate the distance from the node's position to the goal
            distanceToGoal = norm(position - obj.xGoal);
            % Check if it's within the goal threshold
            inGoal = distanceToGoal <= obj.goalThreshold;
        end
    end
end
