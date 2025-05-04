%% データの読み取りと変換

dt = 0.002;                     % サンプリング間隔(規定値) [s]

% 1列目がduty比, 2列目が角度計測値 [rad], 3列目が前サンプリングとの時刻差 [μs]であるシステム同定実験結果(csvファイル)の読み込み
data = readmatrix('identification_experiment_data.csv');  
length = size(data(:, 1), 1);   % データ長の取得

V = data(:, 1) * 9.97;                  % 電圧指令値 [V] (duty比にバッテリー電圧を乗算することで計算)
theta = data(:, 2);                     % 角度計測値 [rad]
dt_measured = data(:, 3) * 10 ^ (-6);   % サンプリング間隔(計測値) [s]

dtheta = zeros(length, 1);              % 角速度 [rad/s] 配列の作成
for loop = 1:length                     % 角度とサンプリング間隔から角速度を計算
    if loop == 1
        dtheta(loop, 1) = 0;
    else
        dtheta(loop, 1) = (theta(loop, 1) - theta(loop - 1, 1)) / dt_measured(loop, 1);
    end 
end

%% システム同定
FR.u = V;       % 電圧が入力
FR.y = dtheta;  % 角速度が出力

FRdata = iddata(FR.y, FR.u, dt);        % IDDATAオブジェクトの作成
FRdata.InputName = 'Voltage';
FRdata.OutputName = 'AngularVelocity';
figure(1), plot(FRdata);                % システム同定実験における入出力のプロット
grid on;

FRdata_det = detrend(FRdata);           % データの平均値を0にする
G2 = tfest(FRdata_det, 2, 0)            % 極が2、零が0のモデルを同定
figure(2), compare(FRdata, G2, 'r');    % システム同定実験の出力と同定したモデルの出力をプロット
grid on;
