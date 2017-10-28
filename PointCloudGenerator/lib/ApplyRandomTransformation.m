function [ptCloud_Q, T] = ApplyRandomTransformation( ptCloud, max_rot, min_t, max_t )
% apply a random rigid transformation to a point cloud to get a roto-translated
% point cloud
% Parameters:
%   ptCloud: point cloud to wich apply the random transformation
%   max_rot: 3D vector of max rotations (ideally 0,2?)
%   min_t/max_t: minimum and maximum translation, in terms of norm

% random rotation with angles in [0 max_rot]
R=eul2rotm([abs(max_rot(1))*rand(), ...
            abs(max_rot(2))*rand(), ...
            abs(max_rot(3))*rand()]);

% random translation 
min_v = abs(min_t);
max_v = abs(max_t);
t=[ rnd_sign()*rnd_in_interval(min_v(1), max_v(1)),...
    rnd_sign()*rnd_in_interval(min_v(2), max_v(2)),...
    rnd_sign()*rnd_in_interval(min_v(3), max_v(3))]';

% compose rigid transformation
T=[R,t;[0 0 0 1]];

tform = affine3d(T');
ptCloud_Q=pctransform(ptCloud,tform);

end

function s = rnd_sign()
s = sign(rand()-1/2);
end

function r = rnd_in_interval(min_v, max_v)
r = min_v + (max_v-min_v)*rand();
end