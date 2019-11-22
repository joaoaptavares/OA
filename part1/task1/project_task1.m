%% Task 1
clear;

A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
B = [0 0; 0 0; 0.1 0; 0 0.1];
E = [1 0 0 0; 0 1 0 0];
w_k = [10 20 30 30 20 10; 10 10 10 0 0 -10];
time = [11 26 31 41 51 61];

p_initial = [0; 5];
p_final = [15; -15];

% comment! v = 0
x_initial = [p_initial; 0; 0];
x_final = [p_final; 0; 0];


T = 81;
K = 6; %waypoints
U_max = 100;
    
lambda = [0.001, 0.01, 0.1, 1, 10, 100, 1000];

signal_changes = zeros(length(lambda));
deviation = zeros(length(lambda),length(w_k));
mean_deviation = zeros(length(lambda));

hh = waitbar(0);

for i = 1:length(lambda)
 
 cvx_begin quiet
     variables u(2, T-1) x(4,T) %state and control signal are the unknowns
     t = 1:T-1;
     cost_function = 0;

     for j = 2:T-1
        cost_function = cost_function + square_pos(norm(u(:,j)- u(:,j-1),2));
     end
     cost_function = cost_function*lambda(i);
     for j = 1:K
        cost_function = cost_function + square_pos(norm(E*x(:,time(j))- w_k(:,j), 2));
     end

     minimize(cost_function);
     %constraints
     subject to
         x(:,1) == x_initial
         x(:,T) == x_final
         for j = 1:T-1
            norm(u(:,j),2) <= U_max
         end
         x(:,t+1) == A*x(:,t) + B*u(:,t)
     
cvx_end

    plt_x = figure;
    plot(w_k(1,1:K),w_k(2,1:K),'s','MarkerEdgeColor','red', 'MarkerSize',12);
    hold on;
    plot(x(1,time(1:K)),x(2,time(1:K)),'o','MarkerEdgeColor','magenta', 'MarkerSize',12);
    plot(x(1,:),x(2,:),'o', 'MarkerEdgeColor','blue', 'MarkerSize',4);
    grid on;
    axis([0 35 -20 15]);
    saveas(plt_x, sprintf('task1_pltx_%d.png', i));
  
    plt_u = figure;
    plot(t-1, u(1,:), 'blue', 'LineWidth', 1.5);
    hold on;
    plot(t-1, u(2,:), 'cyan', 'LineWidth', 1.5);
    grid on;
    leg = legend('$u_1(t)$', '$u_2(t)$');
    set(leg, 'Interpreter','latex', 'FontSize', 12);
    saveas(plt_u, sprintf('task1_pltu_%d.png', i));
    
    % Task 1 - (c)
    for j = 2:T-1
        if norm(u(:,j)-u(:,j-1),2) > 10^(-6)
            signal_changes(i) = signal_changes(i) + 1;
        end    
    end
    
    % Task 1 - (d)
    for j = 1:K
        deviation(i,j) = norm(E*x(:,time(j))- w_k(:,j), 2);
        mean_deviation(i) = mean_deviation(i) + (1/K)*deviation(i,j);
    end
    
    espera = i/length(lambda);
    hh = waitbar(espera);

end

close(hh)