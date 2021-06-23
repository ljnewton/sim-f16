function [smin, ffin] = f16_trimmer(dof,uguess)

%     Run a Nelder-Mead simplex optimizer to find the control inputs for
%     steady flight at a given condition

% dof: degrees of freedom
% uguess: initial guess for trim control inputs

    global x

    s = zeros(6,1); % state to be optimized
    ds = zeros(6,1);

    s(1) = uguess(1);    % throttle
    s(2) = uguess(2);    % elevator
    s(3) = x(2);    % angle of attack (rad)

    ds(1) = 0.2;
    ds(2) = 1;
    ds(3) = 0.02;

    if dof > 3 % include lateral/directional trim
        s(4) = uguess(3);    % aileron
        s(5) = uguess(4);    % rudder
        s(6) = x(3);    % sideslip (rad)

        ds(4) = 1;
        ds(5) = 1;
        ds(6) = 0.02;
    end
    
    % number of iterations
    nc = input('Number of trim iterations (1000 default): ');
    if isempty(nc)
        nc = 1000;
    end

    sigma = -1;
    [smin, ffin, ~] = simplex(@f16_trimcost,s,ds,sigma,nc);

end

function x = f16_trimconstr(x)
    % modify the vector x in order to meet flight constraints
    
    g = 32.17; % ft/sec

    global coord stab gamma turnrate
    
    Vt = x(1);
    alpha = x(2);
    beta = x(3);
    phi = x(4);
    p = x(7);
    q = x(8);
    
    if coord % coordinated turn 
        D = x(2);
        gam = x(5) - x(2);
        if sin(gam) ~= 0 %climbing
            sgocb = sin(gam)/cos(beta);
            x(5) = D + atan(sgocb/sqrt(1-sgocb^2)); % rate of climb
        end
        
        x(4) = atan(turnrate/g * Vt/cos(x(5)));
        
        phi = x(4);
        theta = x(5);
        
        A = [1  sin(phi)*tan(theta) cos(phi)*tan(theta);
            0   cos(phi)            -sin(phi);
            0   sin(phi)/cos(theta) cos(phi)/cos(theta)];
        b = A\[0;0;turnrate];
        x(7) = b(1);
        x(8) = b(2);
        x(9) = b(3);
        
    else % non-turning flight
        x(4) = phi;
        D = x(2);
        if phi ~= 0
            D = -X(2);  %inverted
        end
        if sin(gamma) ~= 0 %climbing
            sgocb = sin(gamma)/cos(beta);
            x(5) = D + atan(sgocb/sqrt(1-sgocb^2)); % rate of climb
        else
            x(5) = D; % level
        end
        x(7) = p;
        x(8) = q;
        if stab     % stability axis roll
            x(9) = p*sin(alpha)/cos(alpha);
        else
            x(9) = 0;   % body-axis roll
        end
    end
end

function cost = f16_trimcost(s)

    % specify cost function for F-16 trim problem

    global x xcgr
    thtl = s(1);
    el = s(2);
    x(2) = s(3);
    ail = s(4);
    rdr = s(5);
    x(3) = s(6);
    x(13) = f16_tgear(thtl);
    x = f16_trimconstr(x);
    xd = f16_dynamics(x,[thtl, el, ail, rdr]',xcgr);
    
    % want velocity, angle of attack, sideslip angle, and angular rates to
    % all be constants. Wind angles can be different from the
    % user-specified vector in the global variable x, but the rates will
    % match the user-sepcified values.
    
    cost = xd(1)^2 + 100*(xd(2)^2 + xd(3)^2) + 10*(xd(7)^2 + xd(8)^2 + xd(9)^2);
end

function [xmin, ymin, k] = simplex(fx,x,dx,sd,m)
    % minimizes fx(x)
    % x is (n x 1)
    % dx contains initial perturbations in x
    % sd set according to tolerance required; when sd < 0 the algorithm
    % calls fx m times
    
    n = length(x);
    n_v = n + 1;
    
    % y: values of the cost function for each perturbation of x
    y = zeros(n_v,1);
    
    % each column of V: different element of x is perturbed
    V = zeros(n,n_v);
    
    % calculate each perturbation of x
    % the first column is unperturbed
    for i = 1:n
        V(i,:) = x(i);
        V(i,i+1) = x(i) + dx(i); % 2
    end
    
    % calculate the cost function at each perturbation
    y0 = fx(x);    
    y(1) = y0;
    
    for j = 2:n_v
        y(j) = fx(V(:,j)); % 3
    end
    
    % k: number of function evaluations
    k = n_v;
    flag = true;
    
    while flag == true
%         disp(yl)
        flag1 = true;
        flag2 = true;

        % find the perturbation with the highest and lowest costs
        yh = max(y);
        nh = find(y == yh);
        yl = min(y);
        nl = find(y == yl);
        
        % yb: average cost
        yb = mean(y);
        
        % sda: standard deviation of the cost 
        sda = sqrt(sum(y - yb)^2/n_v);
        
        if k >= m || sda <= sd
            sd = sda;
            m = k;
            ymin = y(nl);
            xmin = V(:,nl);
            flag = false;
        end
        
        % xc: centroid of all the perturbations other than the highest cost
        xc = mean(V(:,1:n_v ~= nh),2);
        
        % reflected point
        alpha = 1;
        x = xc + alpha*(xc - V(:,nh));
        k = k+1;
        yr = fx(x);
        
        if yr < yl
            % expanded point
            gamma = 2;
            xx = xc + gamma*(x - xc);
            k = k+1;
            ye = fx(xx);
            
            if ye < yr
                y(nh) = ye;
                V(:,nh) = xx;
            else
                y(nh) = yr;
                V(:,nh) = x;
            end
            flag1 = false;
        end

        if flag1 == true
            % 2nd highest value
            y2 = y(nl);
            for j = 1:n_v % 15
                if j ~= nl && j ~= nh && y(j) > y2
                    y2 = y(j);
                end
            end
            
            if yr < yh && flag2 == true
                y(nh) = yr;
                for i = 1:n
                    V(i,nh) = x(i); % 16
                end
                if yr < y2
                    % go to line 4
                    flag2 = false;
                end
            end
            
            if flag2 == true
                % contraction
                rho = 0.5;
                xx = xc + rho*(V(:,nh) - xc);

                k = k+1;
                yc = fx(xx);
                if yc < yh
                    y(nh) = yc;
                    V(:,nh) = xx;
                else
                    % shrink
                    for j = 1:n_v
                        for i = 1:n
                            sigma = 0.5;

                            V(i,j) = V(i,nl) + 0.5*(V(i,j) - V(i,nl));
                            if j ~= nl
                                y(j) = fx(V(:,j));
                            end
                        end
                    end
                    k = k + n;
                end

                % goto line 4
            
            end
            
        end
    
    end % while loop
end