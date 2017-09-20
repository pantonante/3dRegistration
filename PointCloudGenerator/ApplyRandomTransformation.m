function [ptCloud_Q, T] = ApplyRandomTransformation( ptCloud, max_rot, min_t, max_t )

R=eul2rotm([max_rot(1)*randn(), max_rot(1)*randn(), max_rot(1)*randn()]);
t=[min_t(1)+max_t(1)*randn(), min_t(2)+max_t(2)*randn(), min_t(3)+max_t(3)*randn()]';
T=[R,t;[0 0 0 1]];
tform = affine3d(T');
ptCloud_Q=pctransform(ptCloud,tform);

end

