%% パラメータ設定

% モータ-動翼系のシステム同定結果
a = 1462;
b = 53850;
c = 218100;

% 機体系のパラメータ
I =  0.005611385;

% ゲイン設定
K_pm =240;      % 動翼角度制御器のPゲイン
K_dm = 1.5;     % 動翼角度制御器のDゲイン

K_pa = 0.8;     % 目標動翼角度決定器のPゲイン
K_da = 0.07;    % 目標動翼角度決定器のDゲイン


%% ボード線図の表示

% 開ループ伝達関数のボード線図
[A, B, C, D] = tf2ss([c*K_da*K_dm c*(K_pa*K_dm+K_pm*K_da) c*K_pa*K_pm], [I a*I (b+c*K_dm)*I c*K_pm*I 0 0]); % 伝達関数を状態方程式に変換
sys_o = ss(A, B, C, D);        % 状態空間モデルの作成
figure(1), margin(sys_o)    % ボード線図を表示(位相余裕とゲイン余裕も表示)
grid on;

% 閉ループ伝達関数のボード線図
[A, B, C, D] = tf2ss([c*K_da*K_dm c*(K_pa*K_dm+K_pm*K_da) c*K_pa*K_pm], [I a*I (b+c*K_dm)*I c*(K_pm*I+K_da*K_dm) c*(K_pa*K_dm+K_pm*K_da) c*K_pa*K_pm]); % 伝達関数を状態方程式に変換
sys = ss(A, B, C, D);   % 状態空間モデルの作成
figure(2), bode(sys)    % ボード線図を表示
grid on;
