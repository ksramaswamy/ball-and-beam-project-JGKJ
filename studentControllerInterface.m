classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        p_ball_prev = nan;
        theta_prev = nan;

        % param
        g = 9.81;
        r_arm = 0.0254;
        L = 0.4255;
        
        a = 5 * 9.81 * 0.0254 / (7 * 0.4255);
        b = (5 * 0.4255 / 14) * (0.0254 / 0.4255)^2;
        c = (5 / 7) * (0.0254 / 0.4255)^2;
    
        K = 10;
        tau = 0.1;

        % cost weight
        w_x = 1800;
        w_u = 5;
    end

    %% MPC controller
    methods(Access = protected)
        function V_servo = mpcImpl(obj, t, p_ball, theta)

            T = 2;              % [s] time horizon
            dt_mpc = 0.1;       % [s] time step cannot be too small
            N = T/dt_mpc;       % total step size
            
            [p_ball_ref, ~, ~] = get_ref_traj(t:dt_mpc:t+T); % obtian reference in the time window
            
            % x1: p_ball
            % x2: p_ball_dot
            % x3: theta
            % x4: theta_dot
            
            % get current (initial) state, compute dz and dtheta (maybe use
            % some estimator to have a better approximation)
            if(obj.t_prev==-1)
                % just started, assume 0 accel
                p_ball_dot = 0;
                theta_dot = 0;
            else
                dt = t - obj.t_prev;
                p_ball_dot = (p_ball-obj.p_ball_prev)/dt;
                theta_dot = (theta-obj.theta_prev) /dt;
            end
            x0 = [p_ball;p_ball_dot;theta;theta_dot];
        
            % MPC
            % linearized (Jacobian) dynmics system
            d = obj.c*(obj.L/2-x0(1));
            At=[0,1,0,0;
                obj.c*x0(4)^2*cos(x0(3))^2,0,obj.a*cos(x0(3))+d*x0(4)^2*sin(2*x0(3)),-2*d*cos(x0(3))^2*x0(4);
                0,0,0,1;
                0,0,0,-1/obj.tau];
            Bt=[0;0;0;obj.K/obj.tau];
            
            % construct matrices for the quadprog solver
            M = (4+1)*N;
            lb = repmat([-0.19;-inf;-1.047;-inf;-10],N,1);
            ub = repmat([0.19;inf;1.047;inf;10],N,1);
            
            S = [eye(4)+At*dt_mpc,Bt*dt_mpc];
            SS = repmat({S}, 1, N-1);
            Aeq1 = blkdiag(SS{:});
            
            I = [-eye(4),zeros(4,1)];
            II = repmat({I}, 1, N-1);
            Aeq2 = blkdiag(II{:});
            
            Aeq = zeros(4*N,M);
            Aeq(1:4*(N-1),1:(N-1)*5) = Aeq1;
            Aeq(1:4*(N-1),6:M) = Aeq(1:4*(N-1),6:M)+Aeq2;
            Aeq(4*(N-1)+1:end,1:5) = -I;
            beq = zeros(4*N,1);
            beq(4*(N-1)+1:end) = x0;
            
            w = diag([obj.w_x,0,0,0,obj.w_u]);
            ww = repmat({w}, 1, N);
            H = blkdiag(ww{:});
            ref_vec = zeros(M,1);
            for i=1:N
               ref_vec((i-1)*5+1) = p_ball_ref(i);
            end
            f = -ref_vec'*H';
            options = optimoptions('quadprog','Display','off');
            x = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],options);
            V_servo = x(5);

            % update old states
            obj.t_prev =t;
            obj.p_ball_prev = p_ball;
            obj.theta_prev = theta;
        end
    end 

    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d,v_ball,dtheta] = stepController(obj, t, p_ball, theta)
            V_servo = mpcImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            v_ball = 0;
            dtheta = 0;
        end
    end
    
end
