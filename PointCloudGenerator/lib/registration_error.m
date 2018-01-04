function [rot_err, tra_err] = registration_error(T_est, T_gnd)

rot_err = norm(rotm2eul(T_gnd(1:3,1:3)*T_est(1:3,1:3)'));
tra_err = norm(T_gnd(1:3,4)-T_est(1:3,4));

end