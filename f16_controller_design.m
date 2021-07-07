f16_simulink_DAQ


%%

[A, B] = f16_linearize(xtrim,utrim,xcgr);

%%

Alat = A([3 4 6 7 9],[3 4 6 7 9]);

Alont = A([1 2 5 8 13],[1 2 5 8 13]);
Blont = B([1 2 5 8 13],[1 2]);
Cvel = [1 0 0 0 0];

Alon = A([1 2 5 8],[1 2 5 8]);
Blon = B([1 2 5 8],[1 2]);
Cq = [0 0 1 0];
Calpha = [0 1 0 0];
D = [0 0];

% transfer function from throttle command to speed response
[vdTnum, vdTden] = ss2tf(Alont,Blont,Cvel,D,1);
vdT = tf(vdTnum,vdTden);

% transfer function from elevator command to alpha
[alphadenum, alphadeden] = ss2tf(Alon,Blon,Calpha,D,2);
alphade = tf(alphadenum,alphadeden);

% xcgr 0.3
% Kt = 0.439
% Ka = 0

% xcgr 0.35
% Kt = 8.62
% Ka = -5.11
% Kq = -10.34

%%

Ka = -5.11;

% transfer function from elevator command to pitch rate
[qdenum, qdeden] = ss2tf(Alon,Blon,Cq,D,2);
qde = tf(qdenum,qdeden);

alphaq = minreal(alphade/qde);

qde_fb = minreal(feedback(qde,Ka*alphaq));





