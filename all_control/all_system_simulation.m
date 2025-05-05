%% パラメータ設定

% 時刻関係の変数
dt_sim = 0.0001;            % シミュレーション周期 [s]
dt_control = 0.002;         % 制御周期 [s] (dt_simの整数倍で設定する)
dt_ratio = 20;              % dt_controlとdt_simの比
Tf = 1;                     % シミュレーションの終了時間 [s]
N = Tf / dt_sim + 1;        % シミュレーションのループ数
t = linspace(0, Tf, N)';

% 動翼角度シミュ用の変数
theta_r = zeros(N, 1);      % 動翼角度の目標値 [rad]
V = zeros(N, 1);            % モータ電圧 [V]
theta = zeros(N+1, 1);      % 動翼角度 [rad]
theta0 = deg2rad(0);        % 初期角度 [rad] (初期の角速度、角加速度、角躍度はゼロとする)
e = zeros(N, 1);            % 目標角度と実際の角度の偏差
theta(1, 1) = theta0;
dddtheta = zeros(N+1, 1);   % 動翼角躍度 [rad/s^3]
ddtheta = zeros(N+1, 1);    % 動翼角加速度 [rad/s^2]
dtheta = zeros(N+1, 1);     % 動翼角速度 [rad/s]
V_limit = 10;               % 電源電圧 [V]
a = 1462;                   % モータ-動翼系のシステム同定結果
b = 53850;
c = 218100;
K_pm = 240;                 % 動翼角度コントローラのPゲイン
K_dm = 1.5;                 % 動翼角度コントローラのDゲイン

% 全体シミュ用の変数
theta_limit = deg2rad(14);  % 目標動翼角度の制限 [rad] 15degが物理的な制限で、オーバーシュートを考慮しそれよりも小さくしておく。
I = 0.005611385;            % 機体のロール軸まわりの主慣性モーメント [kg m^2]
Mr = zeros(N, 1);           % 目標ロールモーメント(動翼2枚合計) [Nm]
M_roll_d = zeros(N, 1);     % 外乱ロールモーメント(動翼2枚合計) [Nm]
phi_r = zeros(N, 1);        % 目標ロール角 [rad]
phi = zeros(N, 1);          % ロール角 [rad]
phi0 = deg2rad(0);          % 制御開始時のロール角 [rad]
dphi0 = deg2rad(0);         % 制御開始時のロール角速度 [rad/s]
dphi = zeros(N, 1);         % ロール角速度 [rad/s]
ddphi = zeros(N, 1);        % ロール角加速度 [rad/s^2]
phi(1, 1) = phi0;
dphi(1, 1) = dphi0;
va = zeros(N, 1);           % 対気速度
C = 0.000320;               % M=Cv^2θの係数C
K_pa = 0.8;                 % 目標動翼角度決定器のPゲイン
K_da = 0.07;                % 目標動翼角度決定器のDゲイン

%% 時系列シミュレーション
for loop = 1:N
    
    phi_r(loop, 1) = deg2rad(90);               % 目標ロール角は固定値とする
    va(loop, 1) = 100 - 10 * loop * dt_sim;     % 対気速度を設定(100 m/sから30 m/sへ線形に減少するよう設定する)
    
    % 目標動翼角度を決定
    if(mod(loop, dt_ratio) == 1)
        if loop == 1    % 1ループ目
            Mr(loop, 1) = K_pa * (phi_r(loop, 1) - phi0) - K_da * dphi0;
            theta_r(loop, 1) = Mr(loop, 1) / (C * va(loop, 1) ^ 2);
        else 
            Mr(loop, 1) = K_pa * (phi_r(loop, 1) - phi(loop, 1)) - K_da * dphi(loop, 1);
            theta_r(loop, 1) = Mr(loop, 1) / (C * va(loop, 1) ^ 2);
        end
    else
        % 出力を更新するタイミングでない場合は、前ループの値を用いる
        Mr(loop, 1) = Mr(loop-1, 1);
        theta_r(loop, 1) = theta_r(loop-1, 1);
    end

    % 目標動翼角度の制限
    if theta_r(loop, 1) > theta_limit
        theta_r(loop, 1) = theta_limit;
    end
    if theta_r(loop, 1) < -theta_limit
        theta_r(loop, 1) = -theta_limit;
    end

    % 動翼角度制御系の出力(電圧)の計算
    if(mod(loop, dt_ratio) == 1)
        if loop == 1 %1ループ目
            V(loop, 1) = K_pm * (theta_r(loop, 1)-0); %動翼の初期角度は0と仮定 
        else
            V(loop, 1) = K_pm * (theta_r(loop, 1) - theta(loop, 1))...
             + K_dm * ((theta_r(loop, 1) - theta_r(loop-dt_ratio, 1)) / dt_control - (theta(loop, 1) - theta(loop-dt_ratio, 1)) / dt_control);
        end
    else
        % 出力を更新するタイミングでない場合は、前ループの値を用いる
        V(loop, 1) = V(loop-1, 1);
    end

    % モータ電圧の制限
    if V(loop, 1) > V_limit
        V(loop, 1) = V_limit;
    end
    if V(loop, 1) < -V_limit
        V(loop, 1) = -V_limit;
    end
    
    % 次時刻の動翼角躍度、角加速度、角速度、角度の計算
    % 角躍度を計算
    dddtheta(loop+1, 1) = c * V(loop, 1) - a * ddtheta(loop, 1) - b * dtheta(loop, 1);
    % 積分により角加速度、角速度、角度を更新（オイラー法）
    ddtheta(loop+1, 1) = ddtheta(loop, 1) + dddtheta(loop+1, 1) * dt_sim;
    dtheta(loop+1, 1) = dtheta(loop, 1) + ddtheta(loop+1, 1) * dt_sim;
    theta(loop+1, 1) = theta(loop, 1) + dtheta(loop+1, 1) * dt_sim;

    % 次時刻のロール角加速度、角速度、角度の計算
    % ロール角加速度の計算
    ddphi(loop+1, 1) = C * theta(loop, 1) * (va(loop, 1) ^ 2) / I;
    % 積分により角速度、角度を更新（オイラー法）
    dphi(loop+1, 1) = dphi(loop, 1) + ddphi(loop+1, 1) * dt_sim;
    phi(loop+1, 1) = phi(loop, 1) + dphi(loop+1, 1) * dt_sim;
end

%% シミュレーション結果の表示
subplot(3, 1, 1);
plot(t, rad2deg(phi_r), 'k')        % 目標ロール角の表示
hold on;
plot(t, rad2deg(phi(1:N, 1)), 'r')  % ロール角の表示
grid on;
ylim([-10 120]);
xlabel('Time [s]');
ylabel('\phi [deg]');
legend(["目標ロール角" "ロール角"])
set(gca, 'fontsize', 10);

subplot(3, 1, 2);
plot(t, rad2deg(theta_r(1:N, 1)), 'k')  % 目標動翼角度の表示
hold on;
plot(t, theta(1:N, 1)/pi*180, 'r')      % 動翼角度の表示
grid on;
ylim([-16 16]);
xlabel('Time [s]');
ylabel('\theta [deg]');
legend(["目標動翼角度" "動翼角度"])
set(gca, 'fontsize', 10);

subplot(3, 1, 3);
plot(t, V, 'r')                        % モータ電圧の表示
ylim([-11 11]);
grid on;
xlabel('Time [s]');
ylabel('Voltage [V]');
legend("モータ電圧")
set(gca, 'fontsize', 10);
