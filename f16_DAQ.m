addpath('.\F-16 Lookups')

Vt = 300;   %ft/sec

global x xcg coord stab gamma turnrate utrim
x = [Vt 0 0 0 0 0 0 0 0 0 0 10000 0]';
xcg = 0.3;
coord = 0;
stab = 0;
gamma = 0;
turnrate = 0;
dof = 6;

uguess = [0 0 0 0]';

[smin,ffin] = f16_trimmer(dof,uguess);

xtrim = x;
utrim = [smin(1) smin(2) smin(4) smin(5)]';

%%

[thist, xhist] = ode113(@f16_dynamics_fcn,linspace(0,100,1000),xtrim);

% dt = 0.01;
% uhist = repmat(utrim,1,1000)*0;
% thist = dt*(0:999);
% uhist(2,((thist>1) .* (thist<2)>0)) = 5;
% uhist(2,((thist>=2) .* (thist<3)>0)) = -5;
% uhist = uhist + utrim;
% [thist, xhist] = f16_rk4(0.01,xtrim,uhist,xcg);

%%

% u_cmdhist = zeros(4,length(thist));
% for i = 1:length(u_cmdhist)
%     u_cmdhist(:,i) = u_cmd(thist(i));
% end
% 
% figure;
% plot(thist,u_cmdhist)


function [thist, yhist] = f16_rk4(dt,y0,uhist,xcg)
    
    N = length(uhist);
    thist = dt*(0:N-1);
    yhist = zeros(length(y0),N);
    yhist(:,1) = y0;
    
    for n = 1:N-1
        k1 = f16_dynamics(yhist(:,n),uhist(:,n),xcg);
        k2 = f16_dynamics(yhist(:,n)+0.5*dt*k1,0.5*(uhist(:,n)+uhist(:,n+1)),xcg);
        k3 = f16_dynamics(yhist(:,n)+0.5*dt*k2,0.5*(uhist(:,n)+uhist(:,n+1)),xcg);
        k4 = f16_dynamics(yhist(:,n)+dt*k3,uhist(:,n+1),xcg);
        
        yhist(:,n+1) = yhist(:,n) + (1/6)*dt*(k1 + 2*k2 + 2*k3 + k4);
    end
end

function xdot = f16_dynamics_fcn(t,x)
    
    global utrim xcg
    
    u = utrim + u_cmd(t);
    xdot = f16_dynamics(x,u,xcg);
end

function u = u_cmd(t)
    
%     u = [0 0 0 0]';
%     if t > 1 && t < 2
%         u = [0 1*(t-1) 0 0]';
%     elseif t >=2 && t < 3
%         u = [0 -1*(t-3) 0 0]';
%     end
    
    u = [0 0 0 0]';
    if t > 1 && t < 5
        u = [0 -3 0 0]';
    elseif t >=5 && t < 9
        u = [0 3 0 0]';
    end
    
end


