Vt = 450;   %ft/sec

global x xcgr coord stab gamma turnrate utrim
addpath('.\F-16 Lookups\')
x = [Vt 0 0 0 0 0 0 0 0 0 0 10000 0]';
xcgr = 0.3;
coord = 0;
stab = 0;
gamma = 0 * pi/180; % radians
turnrate = 0.3;
dof = 6;

uguess = [0 0 0 0]';

[smin,ffin] = f16_trimmer(dof,uguess);

xtrim = x;
utrim = [smin(1) smin(2) smin(4) smin(5)]';

disp(['Angle of attack: ' num2str(xtrim(2)*180/pi) ...
    ' deg., Final cost: ' num2str(ffin)])

f16_simulink_setup

%% Load Individual Test Case Here

wavestart = 5;
dt = 0.01;

Kp = 0.65;
Kpi = 0.01;
Kt = 0.439;
Kti = 0;
pilot_delay = 0.15;

ratelim_on = 1;

thetaref_amp = 20;
thetaref_per = 4;

pulse_on = 0;
pulsewidth = 0.5;
pulseend = wavestart + pulsewidth;
OL_cmd = 15;

%%

disp('CTRL-C here to terminate DAQ; any key to continue')
pause

%% DAQ

Kp_vec = 0.1:0.15:1;
thetaref_amp_vec = 3:3:21;                % deg
thetaref_per_vec = 4:4:24;    % sec
ratelim_toggle_vec = [0, 1];

count = 1;

footer = '.mat';
model = 'f16_simulink';

pitch_limiter = 70;
sim_T = 60;

for ratelim_on = ratelim_toggle_vec
    for Kp = Kp_vec
        for thetaref_amp = thetaref_amp_vec
           for thetaref_per = thetaref_per_vec
               
                disp(count)
                disp([ratelim_on Kp thetaref_amp thetaref_per])
                load_system(model);
                simdata = sim(model,sim_T);
                
                simdata = time_interp(simdata,dt);

                save(['./F-16 Data/f16_' num2str(count,'%05.f') footer],'simdata')

                % Increment the file name
                count = count + 1;

           end
        end 
    end
end


%% Auxiliary Functions

% Signal limiter to post process data: replaced by early-termination block

% [bwout, unstable] = signal_limiter(out);
% 
% %normalize the time steps
% 
% function [out, unstable] = signal_limiter(out)
%     pitch_limiter = 70;
%     
%     unstable = false;
%     if ~isempty(find(abs(out.xout(:,5))*180/pi > pitch_limiter,1))
%         unstable = true;
%     end
%     
%     window = 1:find(abs(out.xout(:,5))*180/pi > pitch_limiter,1);
%     out.tout = out.tout(window,:);
%     out.thetaref = out.thetaref(window,:);
%     out.xout = out.xout(window,:);
%     out.an = out.an(window,:);
% end

% Time interpolation to make all output signals a fixed time step
% Variable-step solvers are a lot faster! -> use variable-step solver then
% use this function to post-process

function simdata_interp = time_interp(simdata,dt)
    datanames = who(simdata);
    tf = simdata.tout(end);
    tnew = (0:dt:tf)';
    
    for i = 1:length(datanames)
        name = datanames{i};
        data = simdata.(name);
        simdata_interp.(name) = interp1(simdata.tout,data,tnew);
    end
end



