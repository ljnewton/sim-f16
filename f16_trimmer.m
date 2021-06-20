function [smin, ffin] = f16_trimmer(dof,uguess)

%     r2d = 180/pi;

    global x

    s = zeros(6,1);
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
    
    ds = ds*1.5;

    % number of iterations
    nc = input('Number of trim iterations (1000 default): ');
    if isempty(nc)
        nc = 1000;
    end
    
    f0 = 0;
    ffin = 0;

    sigma = -1;
    [smin, ffin, ~] = simplex(@f16_trimcost,dof,s,ds,sigma,nc,f0,ffin);

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

function cost = f16_trimcost(s) %,x,xcg,coord,stab)

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
    
    cost = xd(1)^2 + 100*(xd(2)^2 + xd(3)^2) + 10*(xd(7)^2 + xd(8)^2 + xd(9)^2);
end

function [xmin, ymin, k] = simplex(fx,n,x,dx,sd,m,~,~)
    % minimizes func(x)
    % x is n x 1
    % dx contains initial perturbations in x
    % sd set according to tolerance required; when sigma<0 the algorithm
    % calls func m times

    n_v = n + 1;
        
    xx = zeros(n,1);
    xc = zeros(n,1);
    
    % y: values of the cost function for each perturbation of x
    y = zeros(n_v,1);
    
    % each column of V: different element of x is perturbed
    V = zeros(n,n_v);
    
    % calculate each perturbation of x
    % the first column is unperturbed
    for i = 1:n
        for j = 1:n_v
            V(i,j) = x(i);  % 1
        end
        V(i,i+1) = x(i) + dx(i); % 2
    end
    
    % calculate the cost function at each perturbation
    y0 = fx(x);
    yl = y0;
    
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
        yh = y(1);  % 4
        yl = y(1);
        nh = 1;
        nl = 1;
        for j = 2:n_v
            if y(j) >= yh
                yh = y(j);
                nh = j;     % high index
            elseif y(j) <= yl
                yl = y(j);
                nl = j;     % low index
            end  % 5
        end
        
        % yb: average cost
        yb = y(1);
        for j = 2:n_v
            yb = yb + y(j); % 6
        end
        yb = yb/n_v;
        d = 0;
        for j = 1:n_v
            d = d + (y(j) - yb)^2; % 7
        end
        % sda: standard deviation of the cost 
        sda = sqrt(d/n_v);
        
        if k >= m || sda <= sd
            xmin = 0*x;
            sd = sda;
            m = k;
            ymin = y(nl);
            for i = 1:n
                xmin(i) = V(i,nl); % 8
            end
            flag = false;
        end
        
        % xc: centroid of all the perturbations other than the highest cost
        for i = 1:n
            xc(i) = 0;
            for j = 1:n_v
                if j ~= nh % 9
                    xc(i) = xc(i) + V(i,j);
                end
            end
            xc(i) = xc(i)/n; % 10
        end
        
        % reflected point
        for i = 1:n
            x(i) = xc(i) + 1*(xc(i) - V(i,nh)); % 11
        end

        k = k+1;
        yr = fx(x);
        
        if yr < yl
            % expanded point
            for i = 1:n
                xx(i) = xc(i) + 2*(x(i) - xc(i)); % 12
            end
            k = k+1;
            ye = fx(xx);
            
            if ye < yr
                y(nh) = ye;
                for i = 1:n
                    V(i,nh) = xx(i); % 13
                end
            else
                y(nh) = yr;
                for i = 1:n
                    V(i,nh) = x(i); % 14
                end
            end
            % go to line 4
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

                for i = 1:n
                    xx(i) = xc(i) + 0.5*(V(i,nh) - xc(i)); % 17
                end

                k = k+1;
                yc = fx(xx);
                if yc < yh
                    y(nh) = yc;
                    for i = 1:n
                        V(i,nh) = xx(i); % 18
                    end
                else
%                     disp([num2str(k) ' shrink'])
                    for j = 1:n_v
                        for i = 1:n

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