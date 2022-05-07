classdef studentControllerInterface < matlab.System
    properties (Access = private)
        t_prev = -1;
        theta_d = 0;
        p_ball_prev = nan;
        theta_prev = nan;
        V_servo_prev = 0;
        count = 0;
        V_servo_table = zeros(1,4);
        init = false;
    end

    properties(Constant)
        % sys param
        g = 9.81;
        r_arm = 0.0254;
        L = 0.4255;

        a = 5 * 9.81 * 0.0254 / (7 * 0.4255);
        b = (5 * 0.4255 / 14) * (0.0254 / 0.4255)^2;
        c = (5 / 7) * (0.0254 / 0.4255)^2;
            
        K = 10;
        tau = 0.1;

        % MPC param
        T = 1;                    % [s] time horizon
        dt_mpc = 0.1;             % [s] time step
        N = 10;                   % total step size
        lb = repmat([-0.19;-Inf;-1.047;-Inf;-10],10,1);
        ub = repmat([0.19;Inf;1.047;Inf;10],10,1);

        Bt=[0;0;0;100];     %Bt=[0;0;0;K/tau];
        % cost weight 
        H = kron(eye(10),diag([1800,0,0,0,5])); % kron(eye(N),diag([1800,0,0,0,5]))
    end

    %% MPC controller
    methods(Access = protected)
        function V_servo = stepImpl(obj, t, p_ball, theta)
            coder.updateBuildInfo('addIncludePaths','$(START_DIR)\codegen\lib\sloveCFTOC');
            coder.cinclude('sloveCFTOC_initialize.h');
            coder.cinclude('sloveCFTOC.h');
            coder.cinclude('sloveCFTOC_terminate.h');
            % only use MPC for every 1+4 timestep
%             if(obj.count<4)
%                 V_servo = obj.V_servo_table(obj.count+1);
%                 obj.count = obj.count+1;
%                 return 
%             else
%                 obj.count = 0;
%             end

            % some often-used variable
            n = obj.N;
            M = (4+1)*n;
            n_ = n-1;
            
            p_ball_ref = zeros(1,n+1);
            for i=1:n+1
                [p_ball_ref(i), ~, ~] = get_ref_traj(t+(i-1)*obj.dt_mpc);
            end
             % obtian reference in the time window
            
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
                % some quick fix for t2 and t3 are the same for some unknown reason
                if(dt==0)
                    V_servo = obj.V_servo_prev;
                    return
                end
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
                0,0,0,-10];
            
            % construct matrices for the quadprog solver
            S = [eye(4)+At*obj.dt_mpc,obj.Bt*obj.dt_mpc];
            Aeq1 =kron(eye(n_),S);
            
            I = [-eye(4),zeros(4,1)];
            Aeq2 = kron(eye(n_),I);
            
            Aeq = zeros(4*n,M);
            Aeq(1:4*(n_),1:(n_)*5) = Aeq1;
            Aeq(1:4*(n_),6:M) = Aeq(1:4*(n_),6:M)+Aeq2;
            Aeq(4*(n_)+1:end,1:5) = -I;
            beq = zeros(4*n,1);
            beq(4*(n_)+1:end) = x0;
            
            ref_vec = zeros(M,1);
            for i=1:n
               ref_vec((i-1)*5+1) = p_ball_ref(i);
            end
            f = -ref_vec'*obj.H';
            xx0 = repmat([x0;0],n,1); % intial guess

%             more accurate inital guess
%             AA = zeros(5*N,5);
%             AA(1:4,1:4) = eye(4);
%             for i=2:N
%                 AA(5*i-4:5*i-1,1:4)=mpower(At,i-1);
%             end
%             xx0 = AA*[x0;0];

            %options = optimoptions('quadprog','algorithm','active-set','Display','off','OptimalityTolerance',1e-5,'StepTolerance',1e-5,'ConstraintTolerance',1e-5);
            %x = quadprog(obj.H,f,[],[],Aeq,beq,obj.lb,obj.ub,xx0,options);
            %x = solveCFTOC_mex(obj.H,f,Aeq,beq,obj.lb,obj.ub,xx0);
            V_servo = 0.0;

            coder.ceval('sloveCFTOC_initialize');
            V_servo = coder.ceval('solveCFTOC_mex',obj.H,f,Aeq,beq,obj.lb,obj.ub,xx0);
            coder.ceval('sloveCFTOC_terminate');
%             if(isnan(V_servo))
%                 V_servo = obj.V_servo_prev;
%             else
%                 V_servo = x(5);
%                 % save for the next 2 steps
% %                 obj.V_servo_table(1) = x(10);
% %                 obj.V_servo_table(2) = x(15);
% %                 obj.V_servo_table(3) = x(20);
% %                 obj.V_servo_table(4) = x(25);
%             end

            % update old states
            obj.t_prev =t;
            obj.p_ball_prev = p_ball;
            obj.theta_prev = theta;
            obj.V_servo_prev = V_servo;
        end
    end 

    methods(Access = public)
        function [V_servo, theta_d,v_ball,dtheta] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
            v_ball = 0;
            dtheta = 0;
        end
    end
    
end
