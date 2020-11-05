
clc;clear;
global obs r p_curr
L1 = Link('d', 0,  'a', 0,   'alpha', pi/2);
L2 = Link('d', 0,  'a', 1,   'alpha', 0);
L3 = Link('d', 0,  'a', 1,   'alpha', -pi/2);
L4 = Link('d', 0,  'a', 0,   'alpha', -pi/2);
L5 = Link('d', 0,  'a', 0,   'alpha', pi/2, 'offset', pi/2);
L6 = Link('d', 0,  'a', 0,   'alpha', 0);
L = [L1 L2 L3 L4 L5 L6];
rob = SerialLink(L , 'name', 'arm');
r = 0.2;
axis([-6 2 -3 3 -2 5])

qz = [0 0 pi/2 0 0 0];

T =[

    0.8159   -0.3714   -0.4432   -0.8131
    0.3263    0.9285   -0.1773   -0.3350
    0.4774         0    0.8787   -1.5000
         0         0         0    1.0000];
     
q = rob.ikine6s(T);
time = 0:0.1:5;
[tj] = jtraj(qz, q, time);
idx = round(length(time)/2);
TO = rob.fkine(tj(idx, :));
obs = transl(TO);
a = plot_sphere(obs', r, 'blue');
rob.plot(tj, 'trail' , '.k');

%% Prova algo di ottimizzazione
%{
syms q [1 6]
q_sym = [q1 q2 q3 q4 q5 q6];
w = norm(double(rob.fkine(q_sym)) * [0 0 0 1]' - [obs 1]');
wd = gradient(w, q_sym);
k0 = 1;
p0 = transl(rob.fkine(qz));
q_final = [];
for i = 0:1:length(time)-1
    p = transl(rob.fkine(tj(i+1, :)));
    if i == 0
        p_prev = p;
        q = qz;
    else
        q = tj(i, :);
    end
    
    J = rob.jacob0(q);
    I_n = eye(6,6);
    J_rpi = J' * 1/(J*J');
    
    q0d = k0 .* double(subs(wd, q_sym, q));
    
    ve = (p - p_prev) / 0.1;
    
    q_vel = J_rpi * [ve zeros(1, 3)]' +  (I_n - J_rpi * J) * q0d;
    q_final = [q_final; q + (q_vel' * 0.1)];
    p_prev = p;    
end
%}

%% Prova di ottimizzazione 2 - Algo fatto da me
H = rob.fkine(tj);
pj = transl(H);
pj_opt = zeros(length(time), 3);
opt_flag = false;
for i = 1:1:length(time)-1
    p_curr = pj(i, :);
    p = pj(i+1, :); % Sto un passo avanti
    % Vedo se devo ottimizzare
    if (norm(p - obs) < r+0.1)
        opt_flag = true;
        
    elseif ( norm(p_curr - obs) > r + 0.3) & ( opt_flag == true)
          fprintf( '%s\n', "Fine optimizzaz")
          opt_flag = false;
    end
    
    if (opt_flag == true)
            dis = @(x) norm(x-p);
                if pj_opt(i-1, :) == [0,0, 0]
                    p0 = pj(i, :);
                else
                    p0 = pj_opt(i-1, :);
                end
            y = fmincon(dis, p0, [], [], [], [],[],[],  @nonlin)
            pj(i+1, :)
            pj_opt(i+1, :) = y;
            pj(i+1, :) = y;
    end 
end
th = rob.ikine6s(pj);
rob.plot(th);
function [c, ceq] = nonlin(point)
global obs r p_curr
c(1) = -norm(point - obs) + (r+0.1); 
c(2) = norm(point -obs) - (r+0.6);
c(3) = norm(point - p_curr) - 0.28;
ceq = [];
end
