
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% User config
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
initStdDev = 0.3*20;
maxNbrItrs = 500;
func = @Rosenbrock;
% For example:
%	@Rosenbrock
%	@Ackley
%	@Rastrigin
%   @Sphere
nbrDim = 10; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Execute
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options = CMAOptions();
options = options.set({'sigma', initStdDev,'max iterations', maxNbrItrs});

x0 = rand([1 nbrDim]);
[x, feval, exitFlag] = cma(func, x0, options);