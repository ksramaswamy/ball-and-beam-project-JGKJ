function u = solveCFTOC(H,f,A,b,lb,ub,x0)
    options = optimoptions('quadprog','algorithm','active-set','Display','off','OptimalityTolerance',1e-5,'StepTolerance',1e-5,'ConstraintTolerance',1e-5);
    x = quadprog(H,f,[],[],A,b,lb,ub,x0,options);
    u = x(5);
end