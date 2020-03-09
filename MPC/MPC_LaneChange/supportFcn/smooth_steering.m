function [out] = smooth_steering(in)
% 2019/5/14;
% Smooth function for steering command 
if length(in) < 5
    out = in(end);
else
    out = in(end)*0.2+in(end-1)*0.2+in(end-2)*0.2+in(end-3)*0.2+in(end-4)*0.2;
end

end