%%
% Author: Uriel Martinez-Hernandez
% Date: 7 August 2017
%%

% TEST OF COMMUNICATION BETWEEN MATLAB AND ARDUINO

sPort = serial('COM4');
set(sPort, 'Baudrate', 9600, 'Terminator', 'CR');
disp('Opening serial port');
fopen(sPort);

pause(2);

out1 = cell(1,1);
for i=1:10
    disp('Sending CALIBRATE command');
    fprintf(sPort, '@CALIBRATE');
    disp('Reading output');
    for j=1:2
        out1{j,i} = fscanf(sPort);
    end
    pause(0.1);
end

disp('========');
pause(2);

out2 = cell(1,1);
for i=1:10
    disp('Sending INFO command');
    fprintf(sPort, '@INFO');
    disp('Reading output');
    for j=1:11
        out2{j,i} = fscanf(sPort);
    end
    pause(0.1);
end

fclose(sPort);
delete(sPort);
clear sPort;