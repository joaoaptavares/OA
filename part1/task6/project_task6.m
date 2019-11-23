%% Task 6 - Variation B

clear;

A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
B = [0 0; 0 0; 0.1 0; 0 0.1];
E = [1 0 0 0; 0 1 0 0];
c_k = [10 20 30 30 20 10; 10 10 10 0 0 -10];
time = [11 26 31 41 51 61];

p_initial = [0; 5];
p_final = [15; -15];

% comment! v = 0
x_initial = [p_initial; 0; 0];
x_final = [p_final; 0; 0];

% All the radiuses of the disks are 2
r_disk = 2;

T = 81;
K = 6; %waypoints
U_max = 100;
    
lambda = 10^-1;

signal_changes = 0;
mean_deviation = 0;
 
cvx_begin quiet
     variables u(2, T-1) x(4,T) %comment !
     t = 1:T-1;
     cost_function = 0;

     for j = 2:T-1
        cost_function = cost_function + norm(u(:,j)- u(:,j-1),2);
     end
     cost_function = cost_function*lambda;
     for j = 1:K
        cost_function = cost_function + ...
            max(0,square_pos(norm(E*x(:,time(j)) - c_k(:,j), 2) - r_disk));
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
    n = 1000;
    t_circle = linspace(0,2*pi,n);
    for k=1:K
         x_circle = c_k(1,k) + r_disk*sin(t_circle);
         y_circle = c_k(2,k) + r_disk*cos(t_circle);
         line(x_circle,y_circle, 'Color', 'red');
    end
    hold on;
%     plot(c_k(1,1:K),c_k(2,1:K),'o','MarkerEdgeColor','red','MarkerSize', ...
%         6,'MarkerFaceColor','red');
    hold on;
    plot(x(1,time(1:K)),x(2,time(1:K)),'o','MarkerEdgeColor','magenta','MarkerSize',10);
    plot(x(1,:),x(2,:),'o','MarkerEdgeColor','blue','MarkerSize',4);
    grid on;
    axis([0 35 -15 15]);
    saveas(plt_x,'task6_pltx.png');
  
    plt_u = figure;
    plot(t-1, u(1,:),'blue','LineWidth',1.5);
    hold on;
    plot(t-1, u(2,:),'cyan','LineWidth',1.5);
    grid on;
    leg = legend('$u_1(t)$', '$u_2(t)$');
    set(leg, 'Interpreter','latex','FontSize',12);
    saveas(plt_u,'task6_pltu.png');
    
    % Task 6 - (c)
    for j = 2:T-1
        if norm(u(:,j)-u(:,j-1),2) > 10^(-6)
            signal_changes = signal_changes + 1;
        end    
    end
    
    % Task 6 - (d)
    for j = 1:K
        mean_deviation = mean_deviation + (1/K)*norm(E*x(:,time(j))- c_k(:,j), 2);
    end
    
    signal_changes
    mean_deviation
  

   
    
