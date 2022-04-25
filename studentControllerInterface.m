classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% Class Variables
        t_prev = -1;
        theta_d = 0;
        thetaLast = 0;
        p_ballLast = 0;
        dt = .01;
        v_ball = 0;
        dtheta = 0;
        controllerType = 3;
        inputLast = 0;

        % Notes on Kalman Filter: There's an extra state to account for
        % potential offset on the servo angle measurements. This could
        % happen if the machine isn't setup perfectly and 0 servo angle
        % doesn't correspond to perfectly level. The extra state represents
        % this offset and the Kalman filter estimates what the bias is to
        % correct for it.

        % Kalman Filter Variables
        Pm = diag([.05 .02 .08 .04 .02]);
        xm = [-.19 0 0 0 0]';
        n = 5;
        Evv = diag([.01 .01 .01 .01 0.01]).^2;
        Eww = diag([.005 .002]).^2;

    end
    methods(Access = protected)
        %         function setupImpl(obj)
        %             disp("You can use this function for initializaition.")
        %         end

        function V_servo = stepImpl(obj, t, p_ball, theta)
            % This is the main function called every iteration. You have to implement
            % the controller in this function, but you are not allowed to
            % change the signature of this function.
            % Input arguments:
            %   t: current time
            %   p_ball: position of the ball provided by the ball position sensor (m)
            %
            %   theta: servo motor angle provided by the encoder of the motor (rad)
            % Output:
            %   V_servo: voltage to the servo input.

            %% Read data
            obj.dt = t - obj.t_prev;

            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, ~] = get_ref_traj(t);
            
            %% State Estimation - Unscented Kalman Filter
            %Estimate ball velocity
%             obj.v_ball = (p_ball - obj.p_ballLast)/obj.dt;
%             % Estimate angular velocity of servo
%             obj.dtheta = (theta - obj.thetaLast)/obj.dt;
            % Generate sigma points
            S = chol(obj.n*obj.Pm);
            s = zeros(5,10);
            for i = 1:5
                s(:,i) = obj.xm + S(:,i);
                s(:,5+i) = obj.xm - S(:,i);
            end
            % Calculate prior sigma points
            sp = zeros(5,10);
            for i = 1:obj.n*2
                sp(:,i) = s(:,i) + obj.dynamics(s(:,i),obj.inputLast)*obj.dt;
            end
            % Compute the prior statistics
            xp = mean(sp,2);
            Pp = zeros(5);
            for i = 1:obj.n*2
                Pp = Pp + 1/(2*obj.n)*(sp(:,i) - xp)*(sp(:,i)-xp)' + obj.Evv;
            end

            % Measurement Update
            % Sigma points for measurements
            sz = [sp(1,:);sp(3,:)+sp(5,:)];

            % Measurement Statistics
            zbar = mean(sz,2);
            Pzz = zeros(2);
            for i = 1:obj.n*2
                Pzz = Pzz + 1/(2*obj.n)*(sz(:,i) - zbar)*(sz(:,i)-zbar)' + obj.Eww;
            end

            Pxz = zeros(5,2);
            for i = 1:obj.n*2
                Pxz = Pxz + 1/(2*obj.n)*(sp(:,i) - xp)*(sz(:,i)-zbar)';
            end

            % Apply Kalman Gain
            K = Pxz/Pzz;
            obj.xm = xp + K*([p_ball;theta] - zbar);
            obj.Pm = Pp - K*Pzz*K';

            % Store data in variables for control
            p_ball = obj.xm(1);
            obj.v_ball = obj.xm(2);
            theta = obj.xm(3);
            obj.dtheta = obj.xm(4);

            switch(obj.controllerType)
                case 1
                    %% PD Controller
                    % Decide desired servo angle based on simple proportional feedback.
                    k_p = 2; k_d = 20;
                    obj.theta_d = - k_p * (p_ball - p_ball_ref) - k_d*obj.v_ball;

                    % Make sure that the desired servo angle does not exceed the physical
                    % limit. This part of code is not necessary but highly recommended
                    % because it addresses the actual physical limit of the servo motor.
                    theta_saturation = 56 * pi / 180;
                    obj.theta_d = min(obj.theta_d, theta_saturation);
                    obj.theta_d = max(obj.theta_d, -theta_saturation);

                    % Simple position control to control servo angle to the desired
                    % position.
                    k_servo = 5; kd_servo = 1;
                    V_servo = k_servo * (obj.theta_d - theta) - kd_servo*obj.dtheta;
                case 2
                    %% Sontag Controller
                    x1 = p_ball - (p_ball_ref);
                    x2 = obj.v_ball - (v_ball_ref);
                    x3 = theta;
                    x4 = obj.dtheta;

                    V_servo = ...
-(1.0*((0.418*sin(x3) + x4^2*cos(x3)^2*(0.00255*x1 - 5.42e-4))*(988.0*x1 + 1240.0*x2 + 84.6*x3 + 1.81*x4) - 40.0*x4*(1.29*x1 + 1.81*x2 + 0.534*x3 + 0.0122*x4) + x4*(61.1*x1 + 84.6*x2 + 24.5*x3 + 0.534*x4) + x2*(1410.0*x1 + 988.0*x2 + 61.1*x3 + 1.29*x4) + (((0.418*sin(x3) + x4^2*cos(x3)^2*(0.00255*x1 - 5.42e-4))*(988.0*x1 + 1240.0*x2 + 84.6*x3 + 1.81*x4) - 40.0*x4*(1.29*x1 + 1.81*x2 + 0.534*x3 + 0.0122*x4) + x4*(61.1*x1 + 84.6*x2 + 24.5*x3 + 0.534*x4) + x2*(1410.0*x1 + 988.0*x2 + 61.1*x3 + 1.29*x4))^2 + (77.5*x1 + 109.0*x2 + 32.0*x3 + 0.733*x4)^4)^(1/2)))/(77.5*x1 + 109.0*x2 + 32.0*x3 + 0.733*x4);

                    % Saturate output to conserve energy
                    saturate = 1;
                    V_servo = max(min(V_servo,saturate),-saturate);
                case 3
                    %% LQR Controller
                    K = [12.9099   18.1471    5.3350    0.1222];
                    x1 = p_ball - (p_ball_ref);
                    x2 = obj.v_ball - (v_ball_ref);
                    x3 = theta;
                    x4 = obj.dtheta;

                    V_servo = -K*[x1;x2;x3;x4];
                    % Saturate output to conserve energy
                    saturate = 1;
                    V_servo = max(min(V_servo,saturate),-saturate);
                otherwise
                    V_servo = 0;
            end

            %% Update Class Properties
            obj.t_prev = t;
            obj.thetaLast = theta;
            obj.p_ballLast = p_ball;
            obj.inputLast = V_servo;
        end
    end

    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d,v_ball,dtheta] = stepController(obj, t, p_ball, theta)
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            v_ball = obj.v_ball;
            dtheta = obj.dtheta;
        end
    end

    methods(Access = private)
        function dx = dynamics(~,x,u)
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
            x4 = x(4);

            g = 9.81;
            r_arm = 0.0254;
            L = 0.4255;

            a = 5 * g * r_arm / (7 * L);
            b = (5 * L / 14) * (r_arm / L)^2;
            c = (5 / 7) * (r_arm / L)^2;

            dx = zeros(5, 1);

            % dynamics
            dx(1) = x2;
            dx(2) = a * sin(x3) - b * x4^2 * cos(x3)^2 + c * x1 * x4^2 * cos(x3)^2;
            dx(3) = x4;
            K = 1.5;
            tau = 0.025;
            dx(4) = (- x4 + K * u) / tau;
            dx(5) = 0;
        end
    end

end
