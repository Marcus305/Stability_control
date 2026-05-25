% UDP setup
ip_esp32 = '192.168.1.44';
esp32_port = 4000;

clear hil_node; 
hil_node = udpport("IPv4");
hil_node.Timeout = 1;

% State space matrices
m = 1500; Iz = 2500; lf = 1.2; lr = 1.4; Caf = 60000; Car = 60000; v_lon = 20;
A = [ -(Car + Caf)/(m * v_lon), 0, (Car*lr - Caf*lf)/(m * v_lon) - v_lon;
      0, 0, 1;
      (Car*lr - Caf*lf)/(Iz * v_lon), 0, -(lf^2 * Caf + lr^2 * Car)/(Iz * v_lon) ];
B = [ Caf / m; 0; (lf * Caf) / Iz ];

% HIL
dt = 0.05;          
t_final = 5;        
t = 0:dt:t_final;
N = length(t);

x = zeros(3, N);    
delta_log = zeros(1, N); 

% Initial condition: The car started to skid
x(:, 1) = [0; 0; 0.6]; 

disp('Starting HIL Simulation...');

for k = 1:(N-1)
    current_omega = x(3, k);
    
    tx_str = strrep(sprintf('%.4f', current_omega), ',', '.');
    write(hil_node, tx_str, "string", ip_esp32, esp32_port);
    
    timeout_cnt = 0;
    while hil_node.NumBytesAvailable == 0
        pause(0.001); 
        timeout_cnt = timeout_cnt + 1;
        if timeout_cnt > 1000 
            disp('Warning: ESP32 It took too long to reply!');
            break;
        end
    end
    
    if hil_node.NumBytesAvailable > 0

        data = read(hil_node, hil_node.NumBytesAvailable, "string");
        last_data = data(end); 
        
        last_data = strrep(last_data, ',', '.');
        delta = str2double(last_data);
        
        if isnan(delta)
            delta = 0;
        end
    else
        delta = delta_log(max(1, k-1)); 
    end
    
    if delta > 0.5;  delta = 0.5;  end
    if delta < -0.5; delta = -0.5; end
    
    delta_log(k) = delta;
    
    sys_dyn = @(t_sim, x_sim) A * x_sim + B * delta;
    [~, x_temp] = ode45(sys_dyn, [t(k) t(k+1)], x(:, k));
    
    x(:, k+1) = x_temp(end, :)';
end

disp('HIL Simulation Completed!');
clear hil_node;

figure;
subplot(2,1,1);
plot(t, x(3, :), 'LineWidth', 2, 'Color', 'r');
title('HIL Stability Control - Yaw Rate (\omega)');
ylabel('rad/s'); grid on; hold on;
yline(0, '--k', 'Reference (Straight Car)');

subplot(2,1,2);
plot(t, delta_log, 'LineWidth', 2, 'Color', 'g');
title('ECU (ESP32) Operation - Steering Angle (\delta)');
xlabel('Time (s)'); ylabel('rad'); grid on;