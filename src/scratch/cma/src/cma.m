function [x, feval, exitFlag, bestCost, bestSol, M, sigma] = cma(CostFunction, x0, options)

	if nargin == 3
		if isa(options, 'CMAOptions')
			maxItr =          options.get('max iterations');
			sigma0 =          options.get('sigma');
			lambda =          options.get('population');
			outputErrorTol =  options.get('output tolerance');
			isPlotError =     options.get('should plot error?');
			isPrintBestCost = options.get('should print best cost?');
		else
			error('`options` must be type CMAOptions');
		end
	else
		warning('using default options')
		maxItr = 3;
		sigma0 = 25;
		outputErrorTol  = 1e-6;
		isPlotError = true;
		isPrintBestCost = true;
		lambda;
	end

	%% Problem Settings
	nVar = length(x0);                % Number of Unknown (Decision) Variables
	VarSize = [1 nVar];       % Decision Variables Matrix Size

	%% CMA-ES Settings

	% Population Size (and Number of Offsprings)
	
	if isempty(lambda)
		lambda = (4+round(3*log(nVar)))*10;
	end

	% Number of Parents
	mu = round(lambda/2);

	% Parent Weights
	w = log(mu+0.5)-log(1:mu);
	w = w/sum(w);

	% Number of Effective Solutions
	mu_eff = 1/sum(w.^2);

	% Step Size Control Parameters (c_sigma and d_sigma);
	cs = (mu_eff+2)/(nVar+mu_eff+5);
	ds = 1+cs+2*max(sqrt((mu_eff-1)/(nVar+1))-1,0);
	ENN = sqrt(nVar)*(1-1/(4*nVar)+1/(21*nVar^2));

	% Covariance Update Parameters
	cc = (4+mu_eff/nVar)/(4+nVar+2*mu_eff/nVar);
	c1 = 2/((nVar+1.3)^2+mu_eff);
	alpha_mu = 2;
	cmu = min(1-c1,alpha_mu*(mu_eff-2+1/mu_eff)/((nVar+2)^2+alpha_mu*mu_eff/2));
	hth = (1.4+2/(nVar+1))*ENN;

	%% Initialization

	ps = cell(maxItr,1);
	pc = cell(maxItr,1);
	C  = cell(maxItr,1);
	sigma = cell(maxItr,1);

	ps{1} = zeros(VarSize);
	pc{1} = zeros(VarSize);
	C{1}  = eye(nVar);
	sigma{1} = sigma0;

	empty_individual.Position = [];
	empty_individual.Step = [];
	empty_individual.Cost = [];
	empty_individual.Population = [];

	M = repmat(empty_individual,maxItr,1);
	M(1).Position = x0;
	M(1).Step = zeros(VarSize);
	M(1).Cost = CostFunction(M(1).Position);
	M(1).Population = [];
	
	bestSol = M(1);

	bestCost = zeros(maxItr,1);

	%% CMA-ES Main Loop

	itr = 0;
	while 1
		itr = itr+1;

		% Generate Samples
		population = repmat(empty_individual,lambda,1);
		for i = 1:lambda
			population(i).Step = mvnrnd(zeros(VarSize),C{itr});
			population(i).Position = M(itr).Position+sigma{itr}*population(i).Step;
			population(i).Cost = CostFunction(population(i).Position);

			% Update Best Solution Ever Found
			if population(i).Cost<bestSol.Cost
				bestSol = population(i);
			end
		end

		% Sort Population
		Costs = [population.Cost];
		[~, SortOrder] = sort(Costs);
		population = population(SortOrder);
		M(itr).population = population;

		% Save Results
		bestCost(itr) = bestSol.Cost;

		% Display Results
		if isPrintBestCost
			disp(['Iteration ' num2str(itr) ': Best Cost = ' num2str(bestCost(itr))]);
		end

		% Update Mean
		M(itr+1).Step = 0;
		for j = 1:mu
			M(itr+1).Step = M(itr+1).Step+w(j)*population(j).Step;
		end
		M(itr+1).Position = M(itr).Position+sigma{itr}*M(itr+1).Step;
		M(itr+1).Cost = CostFunction(M(itr+1).Position);
		if M(itr+1).Cost < bestSol.Cost
			bestSol = M(itr+1);
		end

		% Update Step Size
		ps{itr+1} = (1-cs)*ps{itr}+sqrt(cs*(2-cs)*mu_eff)*M(itr+1).Step/chol(C{itr}+0.1*eye(size(C{itr})))';
		sigma{itr+1} = sigma{itr}*exp(cs/ds*(norm(ps{itr+1})/ENN-1))^0.3;

		% Update Covariance Matrix
		if norm(ps{itr+1})/sqrt(1-(1-cs)^(2*(itr+1))) < hth
			hs = 1;
		else
			hs = 0;
		end
		delta = (1-hs)*cc*(2-cc);
		pc{itr+1} = (1-cc)*pc{itr}+hs*sqrt(cc*(2-cc)*mu_eff)*M(itr+1).Step;
		C{itr+1} = (1-c1-cmu)*C{itr}+c1*(pc{itr+1}'*pc{itr+1}+delta*C{itr});
		for j = 1:mu
			C{itr+1} = C{itr+1}+cmu*w(j)*population(j).Step'*population(j).Step;
		end

		% If Covariance Matrix is not Positive Definite or Near Singular
		[V, E] = eig(C{itr+1});
		if any(diag(E) < 0)
			E = max(E,0);
			C{itr+1} = V*E/V;
		end
		
		% Convergence criterion
		if itr == maxItr
			exitFlag = 1;
			break;
		end
	end

	%% Display Results

	if isPlotError
		figure;
		% plot(BestCost, 'LineWidth', 2);
		semilogy(bestCost, 'LineWidth', 2);
		xlabel('Iteration');
		ylabel('Best Cost');
		grid on;
	end

	x = M(end).Position;
	feval = M(end).Cost;
end

