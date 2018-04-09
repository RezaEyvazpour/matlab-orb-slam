classdef CMAOptions < Options
	%CMAOPTIONS Summary of this class goes here
	%   Detailed explanation goes here
	
	properties (Access = protected)
		defaultOptions = {'max iterations',                 15,...
						  'output tolerance',             1e-6,...
						  'sigma',                          25,...
						  'should plot error?',           true,...
						  'should print best cost?',     false,....
						  'population',                     [],... % set by num of variables by default
					     };
	end
	
	methods
		function self = CMAOptions(params, values)
			keys = self.defaultOptions(1:2:end);
			vals = self.defaultOptions(2:2:end);
			self.options = containers.Map(keys, vals);
						
			if nargin == 3
				self = self.set(params,values);
			end
		end
	end
	
end

