% Compute apparent mass & inertia
function [Im, Ii] = ImIiCompute(a, b, c, t)
    a_star = a/b;
    t_star = t/c;
    AR = b/c;
    mA = 0.666 * (1 + 8/3*a_star^2) * t^2 * b;
    mB = 0.267 * (t^2 + 2*a^2*(1-t_star^2))*c;
    mC = 0.785 * sqrt(1 + 2*a_star^2*(1-t_star^2))*AR/(1+AR)*b*c^2;
    IA = 0.055 * AR/(1+AR)*b^3*c^2;
    IB = 0.0308 * AR/(1+AR)*(1 + pi/6*(1+AR)*AR*a_star^2*t_star^2)*b*c^4;
    IC = 0.0555 * (1+8*a_star^2)*t^2*b^3;
    Im = diag([mA, mB, mC]);
    Ii = diag([IA, IB, IC]);

end
