function obj = makeSureIsCell(obj)
	if ~iscell(obj)
		if isnumeric(obj)
			obj = num2cell(obj);
		elseif isa(obj, 'function_handle')
			obj = {obj};
		else
			obj = cellstr(obj);
		end
	end
end