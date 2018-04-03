classdef ourViewSet < viewSet
	properties
		descriptors = {};
	end
	methods 
		function obj = ourViewSet()
			obj = obj@viewSet();
		end
		
		function obj = addView(obj, view, descriptors, varargin)
			obj = addView@viewSet(obj, view, varargin{:});
			obj.descriptors{end+1} = descriptors;
		end
	end
end