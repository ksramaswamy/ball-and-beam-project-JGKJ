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
        controllerType = 2;
        estimatorType = 3;
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
        Evv = diag([0.01 0.01 0.01 0.01 0.01]).^2;
        Eww = diag([.002 .002]).^2;

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

                    obj.xm = [p_ball;obj.v_ball;theta;obj.dtheta;0];
                case 2
                    %% State Estimation - Unscented Kalman Filter

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
                case 3
                    %% Extended Kalman Filter
                    x1 = obj.xm(1);
                    x2 = obj.xm(2);
                    x3 = obj.xm(3);
                    x4 = obj.xm(4);
                    % Linearize and discretize dynamics
                    Ad = [                       0, 1.0,                                                                        0,                                            0
                        0.0025453*x4^2*cos(x3)^2,   0, - 2.0*cos(x3)*sin(x3)*(0.0025453*x1 - 0.00054151)*x4^2 + 0.41829*cos(x3), 2.0*x4*cos(x3)^2*(0.0025453*x1 - 0.00054151)
                        0,   0,                                                                        0,                                          1.0
                        0,   0,                                                                        0,                                        -40.0];

                    Ad = [Ad zeros(4,1);zeros(1,5)];
                    Ad = expm(Ad*obj.dt);
                  
                    % Compute Prior
                    xp = obj.xm + obj.dynamics(obj.xm,obj.inputLast)*obj.dt;
                    Pp = Ad*obj.Pm*Ad' + obj.Evv;

                    % Sensor matrix
                    H = [1 0 0 0 0;0 0 1 0 1];
                    zbar = H*xp; % Expected measurement

                    % Apply Kalman Gain
                    K = Pp*H'/(H*Pp*H' + obj.Eww);
                    obj.xm = xp + K*([p_ball;theta] - zbar);
                    obj.Pm = (eye(5) - K*H)*Pp;

                    % Store data in variables for control
                    p_ball = obj.xm(1);
                    obj.v_ball = obj.xm(2);
                    theta = obj.xm(3);
                    obj.dtheta = obj.xm(4);
            end

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
                    x3 = theta - asin((9007199254740992*a_ball_ref)/3767600918416707); %Feedforward term
%                     x3 = theta;
                    x4 = obj.dtheta;
                    
                    V_servo = ...
-(1.0*(((77.5*x1 + 88.5*x2 + 21.1*x3 + 0.497*x4)^4 + (x4*(58.1*x1 + 65.0*x2 + 15.2*x3 + 0.352*x4) + (0.418*sin(x3) + x4^2*cos(x3)^2*(0.00255*x1 - 5.42e-4))*(652.0*x1 + 606.0*x2 + 65.0*x3 + 1.47*x4) - 40.0*x4*(1.29*x1 + 1.47*x2 + 0.352*x3 + 0.00829*x4) + x2*(1140.0*x1 + 652.0*x2 + 58.1*x3 + 1.29*x4))^2)^(1/2) + x4*(58.1*x1 + 65.0*x2 + 15.2*x3 + 0.352*x4) + (0.418*sin(x3) + x4^2*cos(x3)^2*(0.00255*x1 - 5.42e-4))*(652.0*x1 + 606.0*x2 + 65.0*x3 + 1.47*x4) - 40.0*x4*(1.29*x1 + 1.47*x2 + 0.352*x3 + 0.00829*x4) + x2*(1140.0*x1 + 652.0*x2 + 58.1*x3 + 1.29*x4)))/(77.5*x1 + 88.5*x2 + 21.1*x3 + 0.497*x4);

                    % Saturate output to conserve energy
                    saturate = 5;
                    V_servo = max(min(V_servo,saturate),-saturate);
                case 3
                    %% LQR Controller
                    K = [12.9099   14.7426    3.5210    0.0829];
                    x1 = p_ball - (p_ball_ref);
                    x2 = obj.v_ball - (v_ball_ref);
                    x3 = theta - asin((9007199254740992*a_ball_ref)/3767600918416707); %Feedforward term
                    x4 = obj.dtheta;

                    V_servo = -K*[x1;x2;x3;x4];
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
