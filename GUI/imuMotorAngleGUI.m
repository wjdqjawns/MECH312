%% Serial Port Setting
clear all; close all; clc;
s = serialport("COM13", 115200);
configureTerminator(s,"CR/LF");
flush(s);

%% Graph Setting
windowSize = 30; % 20초 윈도우
samplingRate = 600;
bufferSize = windowSize * samplingRate;

% Figure 생성
h_fig = figure('Position', [100, 100, 800, 500]);

% 하나의 그래프에 두 데이터 플롯
h1 = animatedline('Color', 'b', 'LineStyle', ':', 'LineWidth', 4, ...
    'MaximumNumPoints', bufferSize, 'DisplayName', 'Desired Angle');
hold on;
h2 = animatedline('Color', 'r', 'LineWidth', 2, ...
    'MaximumNumPoints', bufferSize, 'DisplayName', 'Motor Angle');
xlabel('Time [sec]');
ylabel('Angle [Deg]');
title('IMU Angle and Motor Angle Comparison');
grid on;
xlim([0 windowSize]);
% ylim([-1 1]);
legend('show'); % 범례 표시

%% Data Collection and Graph Update
startTime = tic;

try
    while true
        if s.NumBytesAvailable > 0
            data = readline(s);
            values = str2double(split(data, ','));
            
            if length(values) == 2
                currentTime = toc(startTime);
                
                % 그래프 업데이트
                addpoints(h1, currentTime, values(1));
                addpoints(h2, currentTime, values(2));
                
                % 시간축 업데이트
                if currentTime > windowSize
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