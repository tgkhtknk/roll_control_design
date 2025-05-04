%% パラメータ設定

% モータ-動翼系のシステム同定結果
a = 1462;
b = 53850;
c = 218100;

% ゲイン設定
Kpm = 240; % Pゲイン
Kdm = 1.5; % Dゲイン

%% ボード線図の表示

% 開ループ伝達関数のボード線図
[A, B, C, D] = tf2ss([c*Kdm c*Kpm], [1 a b 0]); % 伝達関数を状態方程式に変換
sys_o = ss(A, B, C, D);                         % 状態空間モデルの作成
figure(1), margin(sys_o)                        % ボード線図を表示(位相余裕とゲイン余裕も表示)
grid on;
 
% 閉ループ伝達関数のボード線図
[A, B, C, D] = tf2ss([c*Kdm c*Kpm], [1 a b+c*Kdm c*Kpm]);   % 伝達関数を状態方程式に変換
sys_c = ss(A, B, C, D);                                     % 状態空間モデルの作成
figure(2), bode(sys_c)                                      % ボード線図を表示
grid on;
