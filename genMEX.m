cfg = coder.config('mex');
cfg.IntegrityChecks = false;
cfg.SaturateOnIntegerOverflow = false;
cfg.DynamicMemoryAllocation = 'Off';
% x = solveCFTOC(H,f,A,b,lb,ub,x0)
% H: 5Nx5N
% f: 1x5N
% A: 4Nx5N
% b: 4Nx1
% lb: 5Nx1
% ub: 5Nx1
% x0: 5Nx1
N = 10;
codegen -config cfg solveCFTOC -args {zeros(50,50),zeros(1,50),zeros(40,50),zeros(40,1),zeros(50,1),ones(50,1),zeros(50,1)}
%codegen -config cfg solveCFTOC -args {zeros(100,100),zeros(1,100),zeros(80,100),zeros(80,1),zeros(100,1),ones(100,1),zeros(100,1)}
