classdef (Abstract) Options
	%SOLVEROPTIONS Summary of this class goes here
	%   Detailed explanation goes here
	
	properties (Access = protected)
		options = containers.Map;
	end
	
	methods (Access = public)
		
		function self = set(self, params, values)
			
			if nargin == 2
				paramsAndValues = params;
				paramsAndValues = makeSureIsCell(paramsAndValues);
				params = paramsAndValues(1:2:end);
				values = paramsAndValues(2:2:end);
			else
				params = makeSureIsCell(params);
				values = makeSureIsCell(values);
			end
			
			lenParams = length(params);
			assert(lenParams == length(values));
			for i = 1:lenParams
				param = params{i};
				value = values{i};
				try
					self.options(param) = value;
				catch
					warning('`%s` does not match any parameters', string(param))
				end
			end
		end
		
		function values = get(self, params)
			params = makeSureIsCell(params);
			values = [];
			for i = 1:length(params)
				param = params{i};
				try
					values = [values self.options(param)];
				catch
					warning('`%s` does not match any parameters', string(param))
				end
			end
		end
		
		function keys = getKeys(self)
			keys = self.options.keys;
		end
	end
	
end

