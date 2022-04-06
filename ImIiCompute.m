function [Im, Ii] = ImIiCompute(a, b, c, t)

    a_bar = a/b;
    t_bar = t/c;
    AR = b/c;
    A = 0.666 * (1 + 8/3*a_bar^2) * t^2 * b;
    B = 0.267 * (t^2 + 2*a^2*(1-t_bar^2))*c;
    C = 0.785 * sqrt(1 + 2*a_bar^2*(1-t_bar^2))*AR/(1+AR)*b*c^2;
    D = 0.055 * AR/(1+AR)*b^3*c^2;
    E = 0.0308 * AR/(1+AR)*(1 + pi/6*(1+AR)*AR*a_bar^2*t_bar^2)*b*c^4;
    F = 0.0555 * (1+8*a_bar^2)*t^2*b^3;
    Im = diag([A, B, C]);
    Ii = diag([D, E, F]);

end
