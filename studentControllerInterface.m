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
        controllerType = 1;
        estimatorType = 2;
        inputLast = 0;

        % Notes on Kalman Filter: There's an extra state to account for
        % potential offset on the servo angle measurements. This could
        % happen if the machine isn't setup perfectly and 0 servo angle
        % doesn't correspond to perfectly level. The extra state represents
        % this offset and the Kalman filter estimates what the bias is to
        % correct for it.

        % Kalman Filter Variables
        Pm = diag([.05 .02 .08 .04 .02 2]).^2;
        xm = [-.19 0 0 0 0 2]';
        n = 6;
        Evv = diag([0.01 0.01 0.01 1 1 1]).^2;
        Eww = diag([.002 .002]).^2;

    end
    properties(Constant)
        % System parameters
        g = 9.81;
        rg = .0254;
        L  = .4255;
        K = 1.5;
        tau = .025;
    end
    methods(Access = protected)
        %         function setupImpl(obj)
        %             disp("You can use this function for initializaition.")
        %         end

        function [V_servo,p_ball_meas,v_ball_meas,theta_meas,dtheta_meas] = stepImpl(obj, t, p_ball, theta)
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
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            switch(obj.estimatorType)
                case 1
                    % First order approximation
                    obj.v_ball = (p_ball - obj.p_ballLast)/obj.dt;
                    % Estimate angular velocity of servo
                    obj.dtheta = (theta - obj.thetaLast)/obj.dt;
                    obj.xm = [p_ball;obj.v_ball;theta;obj.dtheta;0;0];
                case 2
                    %% Extended Kalman Filter
                    x1 = obj.xm(1);
                    x2 = obj.xm(2);
                    x3 = obj.xm(3);
                    x4 = obj.xm(4);
                    % Linearize and discretize dynamics
                    Ad = ...
[                              0, 1,                                                                                0,                                                0
(5*obj.rg^2*x4^2*cos(x3)^2)/(7*obj.L^2), 0, (5*obj.g*obj.rg*cos(x3))/(7*obj.L) + (2*obj.rg^2*x4^2*cos(x3)*sin(x3)*((5*obj.L)/14 - (5*x1)/7))/obj.L^2, -(2*obj.rg^2*x4*cos(x3)^2*((5*obj.L)/14 - (5*x1)/7))/obj.L^2
                              0, 0,                                                                                0,                                                1
                              0, 0,                                                                                0,                                           -1/obj.tau];

                    Ad = [Ad zeros(4,2);zeros(2,6)];
                    Ad(4,6) = -1;
                    Ad = expm(Ad*obj.dt);
                  
                    % Compute Prior
                    xp = obj.xm + obj.dynamics(obj.xm,obj.inputLast)*obj.dt;
                    Pp = Ad*obj.Pm*Ad' + obj.Evv;

                    % Sensor matrix
                    H = [1 0 0 0 0 0;0 0 1 0 1 0];
                    zbar = H*xp; % Expected measurement

                    % Apply Kalman Gain
                    KalmanGain = Pp*H'/(H*Pp*H' + obj.Eww);
                    obj.xm = xp + KalmanGain*([p_ball;theta] - zbar);
                    obj.Pm = (eye(6) - KalmanGain*H)*Pp;

                    % Store data in variables for control
                    p_ball = obj.xm(1);
                    obj.v_ball = obj.xm(2);
                    theta = obj.xm(3);
                    obj.dtheta = obj.xm(4);
            end
            x1 = p_ball - (p_ball_ref);
            x2 = obj.v_ball - (v_ball_ref);
            x3 = theta - asin((7*obj.L*a_ball_ref)/(5*obj.g*obj.rg)); %Feedforward term
%             x3 = theta;
            x4 = obj.dtheta;
            switch(obj.controllerType)
                case 1
                    %% Sontag Controller
                    V_servo = ...
-(1.0*(((77.5*x1 + 88.5*x2 + 21.1*x3 + 0.497*x4)^4 + (x4*(58.1*x1 + 65.0*x2 + 15.2*x3 + 0.352*x4) + (0.418*sin(x3) + x4^2*cos(x3)^2*(0.00255*x1 - 5.42e-4))*(652.0*x1 + 606.0*x2 + 65.0*x3 + 1.47*x4) - 40.0*x4*(1.29*x1 + 1.47*x2 + 0.352*x3 + 0.00829*x4) + x2*(1140.0*x1 + 652.0*x2 + 58.1*x3 + 1.29*x4))^2)^(1/2) + x4*(58.1*x1 + 65.0*x2 + 15.2*x3 + 0.352*x4) + (0.418*sin(x3) + x4^2*cos(x3)^2*(0.00255*x1 - 5.42e-4))*(652.0*x1 + 606.0*x2 + 65.0*x3 + 1.47*x4) - 40.0*x4*(1.29*x1 + 1.47*x2 + 0.352*x3 + 0.00829*x4) + x2*(1140.0*x1 + 652.0*x2 + 58.1*x3 + 1.29*x4)))/(77.5*x1 + 88.5*x2 + 21.1*x3 + 0.497*x4);

                    % Saturate output to conserve energy
                    saturate = 10;
                    V_servo = max(min(V_servo,saturate),-saturate);
                case 2
                    %% LQR Controller
                    Klqr = [12.9099   14.7426    3.5210    0.0829];
                    V_servo = -Klqr*[x1;x2;x3;x4];
                    % Saturate output to conserve energy
                    saturate = 10;
                    V_servo = max(min(V_servo,saturate),-saturate);
                otherwise
                    V_servo = 0;
            end

            %% Update Class Properties
            obj.t_prev = t;
            obj.thetaLast = theta;
            obj.p_ballLast = p_ball;
            obj.inputLast = V_servo;
            p_ball_meas = obj.xm(1);
            v_ball_meas = obj.xm(2);
            theta_meas = obj.xm(3);
            dtheta_meas = obj.xm(4);
        end
    end

    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d,xm] = stepController(obj, t, p_ball, theta)
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            xm = obj.xm;
        end
    end

    methods(Access = private)
        function dx = dynamics(obj,x,u)
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
            x4 = x(4);

            a = 5 * obj.g * obj.rg / (7 * obj.L);
            b = (5 * obj.L / 14) * (obj.rg / obj.L)^2;
            c = (5 / 7) * (obj.rg / obj.L)^2;

            dx = zeros(6, 1);

            % dynamics
            dx(1) = x2;
            dx(2) = a * sin(x3) - b * x4^2 * cos(x3)^2 + c * x1 * x4^2 * cos(x3)^2;
            dx(3) = x4;
            dx(4) = (- x4 + obj.K * u) / obj.tau - x(6);
            dx(5) = 0;
            dx(6) = 0;
        end
    end

end
