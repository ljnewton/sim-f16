f16_simulink_DAQ

%%

[A, B] = f16_linearize(xtrim,utrim,xcgr);

Alat = A([3 4 6 7 9],[3 4 6 7 9]);

Alon = A([1 2 5 8 13],[1 2 5 8 13]);
Blon = B([1 2 5 8 13],[1 2]);
Cvel = [1 0 0 0 0];
D = [0 0];

% Alon = [-0.1277 -235 -32.2 -9.51 0.314;
%     -7e-4 -.969 0 .908 -2e-4;
%     0 0 0 1 0;
%     9e-4 -4.56 0 -1.58 0;
%     0 0 0 0 -5];
% Blon = [0 -.244;
%     0 -0.00209;
%     0 0;
%     0 -.199;
%     1087 0];

[vdTnum, vdTden] = ss2tf(Alon,Blon,Cvel,D,1);
vdT = tf(vdTnum,vdTden);