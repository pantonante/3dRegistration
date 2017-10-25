function [ptCloud_Q, T] = ApplyRandomTransformation( ptCloud, max_rot, min_t, max_t )
% apply a random rigid transformation to a point cloud to get a roto-translated
% point cloud

% random rotation with angles in [0 max_rot]
R=eul2rotm([max_rot(1)*rand(), max_rot(2)*rand(), max_rot(3)*rand()]);

% random translation 
t=[ clamp(rnd_sign()*(min_t(1)+max_t(1)*rand()), min_t(1), max_t(1)),...
    clamp(rnd_sign()*(min_t(2)+max_t(2)*rand()), min_t(2), max_t(2)),...
    clamp(rnd_sign()*(min_t(3)+max_t(3)*rand()), min_t(3), max_t(3))]';

% compose rigid transformation
T=[R,t;[0 0 0 1]];

tform = affine3d(T');
ptCloud_Q=pctransform(ptCloud,tform);

end

function s = rnd_sign()
s = sign(rand()-1/2);
end

% project x to [min max]
function y = clamp(x, min, max)
if(x<min)
    y=min;
elseif (x>max)
    y=max;
else
    y=x;
end 
end