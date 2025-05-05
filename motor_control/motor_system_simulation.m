%% パラメータ設定

dt_sim = 0.0001;            % シミュレーション周期[s]
dt_control = 0.002;         % 制御周期 [s] (dt_simの整数倍で設定する)
dt_ratio = 20;              % dt_controlとdt_simの比
Tf = 0.2;                   % シミュレーションの終了時間[s]
N = Tf / dt_sim + 1;        % シミュレーションのループ数
t = linspace(0, Tf, N)';

theta_r = zeros(N, 1);  % 動翼角度の目標値[rad]
V = zeros(N, 1);        % モータ電圧[V]
theta = zeros(N+1, 1);  % 動翼角度[rad]
theta0 = deg2rad(0);    % 初期角度[rad] (初期の角速度、角加速度、角躍度はゼロとする)
e = zeros(N, 1);        % 目標角度と実際の角度の偏差
theta(1, 1) = theta0;

dddtheta = zeros(N+1, 1);   % 動翼角躍度[rad/s^3]
ddtheta = zeros(N+1, 1);    % 動翼角加速度[rad/s^2]
dtheta = zeros(N+1, 1);     % 動翼角速度[rad/s]

v_limit = 10;           % 電源電圧[V]

Kp = 240;               % Pゲイン
Kd = 1.5;               % Dゲイン

% モータ-動翼系のシステム同定結果
a = 1462;
b = 53850;
c = 218100;

%% シミュレーションループ
for loop = 1:N
    theta_r(loop, 1) = deg2rad(15);                 % 動翼角度目標値 15度(固定)
    e(loop, 1) = theta_r(loop, 1) - theta(loop, 1); % 偏差の計算

    % 制御系の出力(電圧)の計算
    if(mod(loop, dt_ratio) == 1)
        % dt_controlごとに出力を更新
        if loop == 1
            % 1ループ目は動翼角速度が不明なため, Pゲインのみ
            V(loop, 1) = Kp * e(loop, 1);
        else
            % PD制御
            V(loop, 1) = Kp * e(loop, 1) + Kd * (e(loop, 1) - e(loop-dt_ratio, 1)) / dt_control;
        end
    else
        % 出力を更新するタイミングでない場合は、前ループの値を用いる
        V(loop, 1) = V(loop-1, 1);
    end

    % Vが電源電圧以上にならないよう制限    
    if V(loop, 1) > v_limit
        V(loop, 1) = v_limit;
    elseif V(loop, 1) < -v_limit
        V(loop, 1) = -v_limit;
    end
    
    % 角躍度を計算
    dddtheta(loop+1, 1) = c * V(loop, 1) - a * ddtheta(loop, 1) - b * dtheta(loop, 1);

    % 積分により角加速度、角速度、角度を更新（オイラー法）
    ddtheta(loop+1, 1) = ddtheta(loop, 1) + dddtheta(loop+1, 1) * dt_sim;
    dtheta(loop+1, 1) = dtheta(loop, 1) + ddtheta(loop+1, 1) * dt_sim;
    theta(loop+1, 1) = theta(loop, 1) + dtheta(loop+1, 1) * dt_sim;           
end

%% シミュレーション結果の表示
subplot(2, 1, 1);
plot(t, V, 'r');    % 電圧のプロット
grid on;
ylim([-11 11]);
xlabel('Time [s]');
ylabel('Voltage [V]');
set(gca, 'fontsize', 10);

subplot(2, 1, 2);
plot(t, rad2deg(theta_r), 'k')          % 目標動翼角度のプロット
hold on;
plot(t, rad2deg(theta(1:N, 1)), 'r')    % 動翼角度のプロット
grid on;
ylim([0 16])
xlabel('Time [s]');
ylabel('\theta [deg]');
legend(["目標動翼角度" "動翼角度"])
set(gca, 'fontsize', 10);