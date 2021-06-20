function [A, B] = f16_linearize(xtrim,utrim,xcgr)

% Create linear approximation of F-16 dynamics about a trimmed state using
% the central difference approximation

n = length(xtrim);
m = length(utrim);

A = zeros(n);
B = zeros(n,m);

base = f16_dynamics(xtrim, utrim, xcgr);

% dfi/dxj
tolmin = 0.01;

disp('Linearizing "A" matrix')
for i = 1:n % each row
    for j = 1:n % each column
        
        alpha = 16;
        flag = false;
        olderiv = inf;
        
        while ~flag
        
            ptb = 0*xtrim;
            ptb(j) = alpha;

            ptb_dyn_plus = f16_dynamics(xtrim + ptb, utrim, xcgr);
            ptb_dyn_minus = f16_dynamics(xtrim - ptb, utrim, xcgr);
            deriv = (ptb_dyn_plus(i) - ptb_dyn_minus(i))/(2*alpha);
            
            change = (olderiv - deriv)/olderiv;
            
            % if the derivative has not converged, smaller perturbations
            if change < tolmin || alpha < 2^-12
                flag = true;
            else
                olderiv = deriv;
                alpha = 0.5*alpha;
            end
        end
        
        if alpha < 2^-12 && deriv ~= 0
            disp(['Linearization failed to converge at A(' num2str(i)...
                ',' num2str(j) ')'])
        end
        A(i,j) = deriv;
    end
end

disp('Linearizing "B" matrix')
for i = 1:n % each row
    for j = 1:m % each column
        
        alpha = 16;
        flag = false;
        olderiv = inf;
        
        while ~flag
        
            ptb = 0*utrim;
            ptb(j) = alpha;

            ptb_dyn = f16_dynamics(xtrim, utrim + ptb, xcgr);
            deriv = (ptb_dyn(i) - base(i))/alpha;
            
            change = (olderiv - deriv)/olderiv;
            
            if change < tolmin || alpha < 2^-12
                flag = true;
            else
                olderiv = deriv;
                alpha = 0.5*alpha;
            end
        end
        
        if alpha < 2^-12 && deriv ~= 0
            disp(['Linearization failed to converge at B(' num2str(i)...
                ',' num2str(j) ')'])
        end
        B(i,j) = deriv;
    end
end

end