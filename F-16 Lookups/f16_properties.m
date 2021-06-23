mass.weight = 20500;    %lb
mass.g = 32.17;         %ft/s^2
mass.Jxx = 9456;        %slug ft^2
mass.Jyy = 55814;       %slug ft^2
mass.Jzz = 63100;       %slug ft^2
mass.Jxz = 982;         %slug ft^2
mass.Hx = 160;          %slug ft^2/s (engine angular momentum)

dims.b = 30;            % ft wingspan
dims.S = 300;           % ft^2 reference area
dims.cbar = 11.32;      % ft mean aerodynamic chord
dims.xcgr = 0.35;
dims.xcg = dims.xcgr*dims.cbar;   % ft reference CG location

ctrl.elev.satlim = 25;      % deg.
% ctrl.elev.ratelim = 60;     % deg/sec
ctrl.elev.ratelim = 30;     % deg/sec
ctrl.elev.lag = 0.0495;     % sec. lag time constant
ctrl.ail.satlim = 21.5;      % deg.
ctrl.ail.ratelim = 80;     % deg/sec
ctrl.ail.lag = 0.0495;     % sec. lag time constant
ctrl.rdr.satlim = 30;      % deg.
ctrl.rdr.ratelim = 120;     % deg/sec
ctrl.rdr.lag = 0.0495;     % sec. lag time constant

