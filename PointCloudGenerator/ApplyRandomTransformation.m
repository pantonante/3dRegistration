function [ptCloud_Q, T] = ApplyRandomTransformation( ptCloud, max_rot, min_t, max_t )

R=eul2rotm([max_rot(1)*rand(), max_rot(2)*rand(), max_rot(3)*rand()]);
t=[ clamp(min_t(1)+max_t(1)*rand(), min_t(1), max_t(1)),...
    clamp(min_t(2)+max_t(2)*rand(), min_t(2), max_t(2)),...
    clamp(min_t(3)+max_t(3)*rand(), min_t(3), max_t(3))]';
T=[R,t;[0 0 0 1]];

tform = affine3d(T');
ptCloud_Q=pctransform(ptCloud,tform);

end

function y = clamp(x, min, max)
if(x<min)
    y=min;
elseif (x>max)
    y=max;
else
    y=x;
end 
end