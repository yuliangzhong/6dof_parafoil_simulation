% guidance guess: control inputs = 0
function guidance0 = GuidanceGuess(N, init_pos)
    h = -init_pos(3);
    hs = linspace(h,0,N);
    guidance0 = [zeros(2,N);
                 hs;
                 zeros(2,N)];
end