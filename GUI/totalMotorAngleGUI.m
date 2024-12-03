%% Serial Port Setting
clear all; close all; clc;
s = serialport("COM13", 115200);
configureTerminator(s,"CR/LF");
flush(s);

%% Graph Setting
windowSize = 60; % 20초 윈도우
samplingRate = 600;
bufferSize = windowSize * samplingRate;

h_fig = figure('Position', [100, 100, 800, 500]);

% 왼쪽에 큰 메인 그래프 배치
h1 = animatedline('Parent',subplot(1,2,1),'Color', 'r', 'LineWidth', 2, ...
    'MaximumNumPoints', bufferSize, 'DisplayName', 'Motor Angle');
hold on;
h2 = animatedline('Color', 'b', 'LineStyle', ':', 'LineWidth', 4, ...
    'MaximumNumPoints', bufferSize, 'DisplayName', 'Desired Angle');
subplot(1,2,1);
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('Total System Control Angle Comparison');
grid on;
xlim([0 windowSize]);
legend('show');
hold off;

% 오른쪽에 IMU 센서 데이터 그래프
h3 = animatedline('Parent',subplot(2,2,2),'Color', 'g', 'LineWidth', 2, ...
    'MaximumNumPoints', bufferSize, 'DisplayName', 'IMU Angle');
subplot(2,2,2);
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('IMU Sensor Data');
grid on;
xlim([0 windowSize]);

% 오른쪽 아래에 Rotary Sensor 데이터 그래프
h4 = animatedline('Parent',subplot(2,2,4),'Color', 'm', 'LineWidth', 2, ...
    'MaximumNumPoints', bufferSize, 'DisplayName', 'Rotary Angle');
subplot(2,2,4);
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('Rotary Sensor Data');
grid on;
xlim([0 windowSize]);

%% Data Collection and Graph Update
startTime = tic;

try
    while true
        if s.NumBytesAvailable > 0
            data = readline(s);
            values = str2double(split(data, ','));
            
            if length(values) == 4 % Desired Angle, Motor Angle, IMU Angle, Rotary Angle
                currentTime = toc(startTime);
                
                % 그래프 업데이트
                addpoints(h1, currentTime, values(1));
                addpoints(h2, currentTime, values(2));
                addpoints(h3, currentTime, values(3));
                addpoints(h4, currentTime, values(4));
                
                % 시간축 업데이트
                if currentTime > windowSize
                    subplot(1,2,1);
                    xlim([currentTime-windowSize currentTime]);
                    subplot(2,2,2);
                    xlim([currentTime-windowSize currentTime]);
                    subplot(2,2,4);
                    xlim([currentTime-windowSize currentTime]);
                end
                
                drawnow limitrate;
            end
        end
    end
catch e
    disp(['Error: ', e.message]);
end

%% Cleanup
clear s;
close all;