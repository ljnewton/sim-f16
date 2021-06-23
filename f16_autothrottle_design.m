f16_simulink_DAQ


%%

[A, B] = f16_linearize(xtrim,utrim,xcgr);

Alat = A([3 4 6 7 9],[3 4 6 7 9]);

Alon = A([1 2 5 8 13],[1 2 5 8 13]);
Blon = B([1 2 5 8 13],[1 2]);
Cvel = [1 0 0 0 0];
D = [0 0];

% transfer function from throttle command to speed response
[vdTnum, vdTden] = ss2tf(Alon,Blon,Cvel,D,1);
vdT = tf(vdTnum,vdTden);