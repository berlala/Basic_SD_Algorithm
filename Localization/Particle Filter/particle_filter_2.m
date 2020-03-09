
N = 100;  
Q = 5;      
R = 5;      
T = 10;     
theta = pi/T;       
distance = 80/T;    
WorldSize = 100;    
X = zeros(2, T);  
Z = zeros(2, T);    
P = zeros(2, N);    
PCenter = zeros(2, T);  
w = zeros(N, 1);         
err = zeros(1,T);     
X(:, 1) = [50; 20];    
Z(:, 1) = [50; 20] + wgn(2, 1, 10*log10(R));    


for i = 1 : N
    P(:, i) = [WorldSize*rand; WorldSize*rand];
    dist = norm(P(:, i)-Z(:, 1));     
    w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R);   
end
PCenter(:, 1) = sum(P, 2) / N;     
%%
err(1) = norm(X(:, 1) - PCenter(:, 1));     
figure(1);
set(gca,'FontSize',10);
hold on
plot(X(1, 1), X(2, 1), 'r.', 'markersize',30)   
axis([0 100 0 100]);
plot(P(1, :), P(2, :), 'kx', 'markersize',5);   
plot(PCenter(1, 1), PCenter(2, 1), 'b.', 'markersize',25); 
legend('True State', 'Particles', 'The Center of Particles');
title('Initial State');
hold off

%%

for k = 2 : T


    X(:, k) = X(:, k-1) + distance * [(-cos(k * theta)); sin(k * theta)] + wgn(2, 1, 10*log10(Q));     
    Z(:, k) = X(:, k) + wgn(2, 1, 10*log10(R));     



    for i = 1 : N
        P(:, i) = P(:, i) + distance * [-cos(k * theta); sin(k * theta)] + wgn(2, 1, 10*log10(Q));
        dist = norm(P(:, i)-Z(:, k));     
        w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R);   
    end
    wsum = sum(w);
    for i = 1 : N
        w(i) = w(i) / wsum;
    end


    for i = 1 : N
        wmax = 2 * max(w) * rand;  
        index = randi(N, 1);
        while(wmax > w(index))
            wmax = wmax - w(index);
            index = index + 1;
            if index > N
                index = 1;
            end          
        end
        P(:, i) = P(:, index);     
    end

    PCenter(:, k) = sum(P, 2) / N;      

    
    err(k) = norm(X(:, k) - PCenter(:, k));     

    figure(2);
    set(gca,'FontSize',12);
    clf;
    hold on
    plot(X(1, k), X(2, k), 'r.', 'markersize',50);  
    axis([0 100 0 100]);
    plot(P(1, :), P(2, :), 'k.', 'markersize',5);   
    plot(PCenter(1, k), PCenter(2, k), 'b.', 'markersize',25); 
    legend('True State', 'Particle', 'The Center of Particles');
    hold off
    pause(0.1);
end

%%
figure(3);
set(gca,'FontSize',12);
plot(X(1,:), X(2,:), 'r', Z(1,:), Z(2,:), 'g', PCenter(1,:), PCenter(2,:), 'b-');
axis([0 100 0 100]);
legend('True State', 'Measurement', 'Particle Filter');
xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20);

%%
figure(4);
set(gca,'FontSize',12);
plot(err,'.-');
xlabel('t', 'FontSize', 20);
title('The err');

%%
%ref: https://blog.csdn.net/shanglianlm/article/details/47445909