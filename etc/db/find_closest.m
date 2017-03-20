function [val, idx] = find_closest(f,x)
    [val, idx] = min(abs(f-x));
end

