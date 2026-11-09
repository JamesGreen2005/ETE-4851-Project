function build_pi_motor_imu(doBuild)
% Builds only. Never flashes the board or opens the serial port.
if nargin==0, doBuild=true; end
taskDir=fileparts(mfilename('fullpath'));
imuDir=fullfile(fileparts(taskDir),'motor_imu');
addpath(taskDir,imuDir);
cd(taskDir);
test_pi_motor;
test_motor_imu;
model='Motor_Pi_IMU';
if bdIsLoaded(model), close_system(model,0); end
if bdIsLoaded('Motor_IMU_Test')
    error('Save and close Motor_IMU_Test before rebuilding this separate copy.');
end
load_system(fullfile(imuDir,'Motor_IMU_Test.slx'));
save_system('Motor_IMU_Test',fullfile(taskDir,[model '.slx']));

% Remove automatic-start sources and their output connections.
for name={'Step','Step1'}
    block=oneBlock(model,name{1});
    ports=get_param(block,'PortHandles');
    for port=ports.Outport
        line=get_param(port,'Line');
        if line~=-1, delete_line(line); end
    end
    delete_block(block);
end
reverse=oneBlock(model,'Constant1');
assert(strcmp(get_param(reverse,'Value'),'0'),'Reverse input must remain zero.');

imuRead=oneBlock(model,'UART/USART Read');
piRead=[model '/Pi Commands USB'];
add_block(imuRead,piRead,'UARTModule','USART3','DataType','uint8', ...
    'DataLength','1','BlockingMode','off','OutputLength','on', ...
    'OutputStatus','on','SampleTime','0.001','Position',[40 30 195 125]);

parser=[model '/Pi Motor Command'];
add_block('simulink/User-Defined Functions/MATLAB Function',parser, ...
    'Position',[260 25 445 165]);
root=sfroot;
chart=root.find('-isa','Stateflow.EMChart','Path',parser);
chart.Script=sprintf(['function [duty,enable,armed,commandError] = command(data,length,status)\n' ...
    '[duty,enable,armed,commandError] = pi_motor_command(data,length,status);\nend\n']);
for k=1:3
    add_line(model,sprintf('Pi Commands USB/%d',k),sprintf('Pi Motor Command/%d',k),'autorouting','on');
end
set_param(oneBlock(model,'PWM Output'),'Position',[525 25 685 115]);
set_param(oneBlock(model,'Digital Port Write'),'Position',[525 155 685 215]);
set_param(oneBlock(model,'Digital Port Write1'),'Position',[750 155 910 215]);
set_param(reverse,'Position',[470 95 500 125]);
add_line(model,'Pi Motor Command/1','PWM Output/1','autorouting','on');
add_line(model,'Pi Motor Command/2','Digital Port Write/1','autorouting','on');
add_line(model,'Pi Motor Command/2','Digital Port Write1/1','autorouting','on');

formatter=oneBlock(model,'Readable Angles');
chart=root.find('-isa','Stateflow.EMChart','Path',formatter);
chart.Script=sprintf(['function text = formatAngles(heading,roll,pitch,calibration,valid,errorCode,armed,duty,commandError)\n' ...
    'text = pi_motor_text(heading,roll,pitch,calibration,valid,errorCode,armed,duty,commandError);\nend\n']);
set_param(formatter,'Position',[750 350 930 600]);
add_line(model,'Pi Motor Command/3','Readable Angles/7','autorouting','on');
add_line(model,'Pi Motor Command/1','Readable Angles/8','autorouting','on');
add_line(model,'Pi Motor Command/4','Readable Angles/9','autorouting','on');

data=get_param(model,'CoderTargetData');
data.STM32CubeMX.ProjectFile=fullfile(taskDir,'hardware','motortest.ioc');
data.Runtime.BuildAction='Build';
data.USART.USART3TransmitMode='Buffered mode using TXE interrupt';
data.USART.USART3ReceiveMode='Buffered mode using RXNE interrupt';
set_param(model,'CoderTargetData',data);
set_param(model,'InitFcn',sprintf('addpath(''%s'',''%s'');', ...
    strrep(taskDir,'''',''''''),strrep(imuDir,'''','''''')));
save_system(model,fullfile(taskDir,[model '.slx']));
set_param(model,'SimulationCommand','update');
save_system(model);
fprintf('MODEL_SAVED: %s\n',fullfile(taskDir,[model '.slx']));
if doBuild
    slbuild(model);
    fprintf('BUILD_ONLY_COMPLETE: boot disabled; no firmware upload requested.\n');
end
close_system(model,0);
end

function block=oneBlock(model,name)
blocks=find_system(model,'SearchDepth',1,'Type','Block','Name',name);
assert(numel(blocks)==1,'Expected one block named %s.',name);
block=blocks{1};
end
