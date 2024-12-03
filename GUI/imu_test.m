%% Serial Port Setting
clear all; close all; clc;
s = serialport("COM18", 115200);
configureTerminator(s,"CR/LF");
flush(s);

%% Graph Setting
windowSize = 60; % 60초 윈도우
samplingRate = 1000;
bufferSize = windowSize * samplingRate;

% Figure 속성 최적화
h_fig = figure('Position', [100, 100, 800, 500], ...
    'DoubleBuffer', 'on', ... % 더블 버퍼링 활성화
    'GraphicsSmoothing', 'off'); % 그래픽 스무딩 비활성화

% 왼쪽에 큰 메인 그래프 배치
subplot(1,2,1);
h1 = plot(NaN, NaN, 'r', 'LineWidth', 2, 'DisplayName', 'Motor Angle');
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('Total System Control Angle Comparison');
grid on;
xlim([0 windowSize]);
ylim([-0.5, 0.5]);

% 오른쪽에 IMU 센서 데이터 그래프
subplot(2,2,2);
h2 = plot(NaN, NaN, 'g', 'LineWidth', 2, 'DisplayName', 'IMU Angle');
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('IMU Sensor Data');
grid on;
xlim([0 windowSize]);

% 오른쪽 아래에 Rotary Sensor 데이터 그래프
subplot(2,2,4);
h3 = plot(NaN, NaN, 'm', 'LineWidth', 2, 'DisplayName', 'Rotary Angle');
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('Rotary Sensor Data');
grid on;
xlim([0 windowSize]);
ylim([-0.5, 0.5]);

% 데이터 버퍼 초기화
timeData = zeros(1, bufferSize);
angleData1 = zeros(1, bufferSize);
angleData2 = zeros(1, bufferSize);
angleData3 = zeros(1, bufferSize);
dataIndex = 1;

%% Data Collection and Graph Update
startTime = tic;

try
    while true
        if s.NumBytesAvailable > 0
            data = readline(s);
            values = str2double(split(data, ','));
            
            if length(values) == 3
                currentTime = toc(startTime);
                % 버퍼에 데이터 저장
                timeData(dataIndex) = currentTime;
                angleData1(dataIndex) = values(1);
                angleData2(dataIndex) = values(2);
                angleData3(dataIndex) = values(3);
                
                % 10개의 데이터마다 그래프 업데이트
                if mod(dataIndex, 10) == 0
                    validIdx = 1:dataIndex;
                    
                    % 메인 그래프 업데이트
                    subplot(1,2,1);
                    set(h1, 'XData', timeData(validIdx), 'YData', angleData1(validIdx));
                    
                    % IMU 그래프 업데이트
                    subplot(2,2,2);
                    set(h2, 'XData', timeData(validIdx), 'YData', angleData2(validIdx));
                    
                    % Rotary 그래프 업데이트
                    subplot(2,2,4);
                    set(h3, 'XData', timeData(validIdx), 'YData', angleData3(validIdx));
                    
                    % 시간축 업데이트
                    if currentTime > windowSize
                        subplot(1,2,1); xlim([currentTime-windowSize currentTime]);
                        subplot(2,2,2); xlim([currentTime-windowSize currentTime]);
                        subplot(2,2,4); xlim([currentTime-windowSize currentTime]);
                    end
                    
                    drawnow limitrate;
                end
                
                % 버퍼 관리
                dataIndex = dataIndex + 1;
                if dataIndex > bufferSize
                    % 버퍼가 가득 차면 절반만큼 이동
                    halfBuffer = bufferSize/2;
                    timeData(1:halfBuffer) = timeData(halfBuffer+1:end);
                    angleData1(1:halfBuffer) = angleData1(halfBuffer+1:end);
                    angleData2(1:halfBuffer) = angleData2(halfBuffer+1:end);
                    angleData3(1:halfBuffer) = angleData3(halfBuffer+1:end);
                    angleData4(1:halfBuffer) = angleData4(halfBuffer+1:end);
                    dataIndex = halfBuffer + 1;
                end
            end
        end
    end
catch e
    disp(['Error: ', e.message]);
end

%% Cleanup
clear s;
close all;