function build_motor_imu(doBuild)
% Build a separate model. Does NOT deploy, connect, or flash the Nucleo.
if nargin == 0, doBuild = true; end
taskDir = fileparts(mfilename('fullpath'));
addpath(taskDir);
cd(taskDir);
test_motor_imu;
original = 'C:/Users/jpgre/OneDrive/Desktop/Books/ETE 3501/Motor_Test.slx';
model = 'Motor_IMU_Test';
target = fullfile(taskDir,[model '.slx']);
if bdIsLoaded(model), close_system(model,0); end
if bdIsLoaded('Motor_Test')
    error('Close Motor_Test after saving your work, then run this script again.');
end
load_system(original);
save_system('Motor_Test',target);
% save_system renames the in-memory copy; original disk file is untouched.

readBlock = oneBlock(model,'UART/USART Read');
writeBlock = oneBlock(model,'UART/USART Write');
pwm = oneBlock(model,'PWM Output');
gpio1 = oneBlock(model,'Digital Port Write');
gpio2 = oneBlock(model,'Digital Port Write1');
motorStep = oneBlock(model,'Step');
motorEnable = oneBlock(model,'Step1');
motorReverse = oneBlock(model,'Constant1');
% Keep a snapshot so model construction cannot silently change motor commands.
beforeMotor = {get_param(motorStep,'Time'),get_param(motorStep,'Before'), ...
    get_param(motorStep,'After'),get_param(motorEnable,'Before'), ...
    get_param(motorEnable,'After'),get_param(motorReverse,'Value')};

% All top-level connections are reconstructed; motor parameters stay unchanged.
lines = find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
for k=1:numel(lines)
    if ishandle(lines(k)), delete_line(lines(k)); end
end
oldFunction = oneBlock(model,'MATLAB Function');
delete_block(oldFunction);
delete_block(oneBlock(model,'Constant'));

% Clone working UART blocks, then use enabled subsystems to avoid empty writes.
imuSend = [model '/IMU Send'];
add_block('simulink/Ports & Subsystems/Subsystem',imuSend,'Position',[530 330 690 410]);
Simulink.SubSystem.deleteContents(imuSend);
add_block(writeBlock,[imuSend '/UART Write'],'UARTModule','USART6', ...
    'LengthOfDataToSend','on','BlockingMode','off','OutputStatus','off', ...
    'Position',[210 55 350 135]);
add_block('simulink/Ports & Subsystems/In1',[imuSend '/Bytes'],'Position',[35 65 65 85]);
add_block('simulink/Ports & Subsystems/In1',[imuSend '/Count'],'Position',[35 115 65 135]);
add_block('simulink/Ports & Subsystems/Enable',[imuSend '/Enable'],'Position',[110 10 130 30]);
add_line(imuSend,'Bytes/1','UART Write/1');
add_line(imuSend,'Count/1','UART Write/2');

usbSend = [model '/USB Telemetry Send'];
add_block('simulink/Ports & Subsystems/Subsystem',usbSend,'Position',[970 345 1135 420]);
Simulink.SubSystem.deleteContents(usbSend);
add_block(writeBlock,[usbSend '/UART Write'],'UARTModule','USART3', ...
    'LengthOfDataToSend','off','BlockingMode','off','OutputStatus','off', ...
    'Position',[160 65 300 130]);
add_block('simulink/Ports & Subsystems/In1',[usbSend '/Text'],'Position',[35 85 65 105]);
add_block('simulink/Ports & Subsystems/Enable',[usbSend '/Enable'],'Position',[100 15 120 35]);
add_line(usbSend,'Text/1','UART Write/1');
delete_block(writeBlock);

set_param(readBlock,'UARTModule','USART6','DataType','uint8','DataLength','1', ...
    'BlockingMode','off','OutputLength','on','OutputStatus','on', ...
    'SampleTime','0.001','Position',[60 330 215 420]);
controller = [model '/BNO055 Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function',controller, ...
    'Position',[300 330 470 555]);
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',controller);
chart.Script = sprintf(['function [tx,count,heading,roll,pitch,valid,calibration,errorCode] = controller(data,length,status)\n' ...
    '[tx,count,heading,roll,pitch,valid,calibration,errorCode] = bno055_controller(data,length,status);\nend\n']);
add_line(model,[blockName(readBlock) '/1'],'BNO055 Controller/1','autorouting','on');
add_line(model,[blockName(readBlock) '/2'],'BNO055 Controller/2','autorouting','on');
add_line(model,[blockName(readBlock) '/3'],'BNO055 Controller/3','autorouting','on');
add_line(model,'BNO055 Controller/1','IMU Send/1','autorouting','on');
add_line(model,'BNO055 Controller/2','IMU Send/2','autorouting','on');
add_line(model,'BNO055 Controller/2','IMU Send/Enable','autorouting','on');

formatter = [model '/Readable Angles'];
add_block('simulink/User-Defined Functions/MATLAB Function',formatter, ...
    'Position',[735 360 900 550]);
chart = root.find('-isa','Stateflow.EMChart','Path',formatter);
chart.Script = sprintf(['function text = formatAngles(heading,roll,pitch,calibration,valid,errorCode)\n' ...
    'text = bno055_text(heading,roll,pitch,calibration,valid,errorCode);\nend\n']);
for k=1:3
    add_line(model,sprintf('BNO055 Controller/%d',k+2),sprintf('Readable Angles/%d',k),'autorouting','on');
end
add_line(model,'BNO055 Controller/7','Readable Angles/4','autorouting','on');
add_line(model,'BNO055 Controller/6','Readable Angles/5','autorouting','on');
add_line(model,'BNO055 Controller/8','Readable Angles/6','autorouting','on');
add_line(model,'Readable Angles/1','USB Telemetry Send/1','autorouting','on');
add_block('simulink/Sources/Pulse Generator',[model '/Telemetry 10 Hz'], ...
    'PulseType','Sample based','Amplitude','1','Period','100','PulseWidth','1', ...
    'PhaseDelay','0','SampleTime','0.001','Position',[750 270 865 310]);
add_line(model,'Telemetry 10 Hz/1','USB Telemetry Send/Enable','autorouting','on');

led = [model '/IMU Fresh Data LED'];
add_block(gpio1,led,'PortName','GPIOB','PinNumber','7', ...
    'Position',[530 485 690 555]);
add_line(model,'BNO055 Controller/6','IMU Fresh Data LED/1','autorouting','on');

% Reconnect the unchanged motor drive blocks.
set_param(motorStep,'Position',[65 65 95 95]);
set_param(motorReverse,'Position',[65 115 95 145]);
set_param(motorEnable,'Position',[65 200 95 230]);
set_param(pwm,'Position',[280 55 440 150]);
set_param(gpio1,'Position',[280 185 440 245]);
set_param(gpio2,'Position',[500 185 660 245]);
add_line(model,'Step/1','PWM Output/1','autorouting','on');
add_line(model,'Constant1/1','PWM Output/2','autorouting','on');
add_line(model,'Step1/1','Digital Port Write/1','autorouting','on');
add_line(model,'Step1/1','Digital Port Write1/1','autorouting','on');
afterMotor = {get_param(motorStep,'Time'),get_param(motorStep,'Before'), ...
    get_param(motorStep,'After'),get_param(motorEnable,'Before'), ...
    get_param(motorEnable,'After'),get_param(motorReverse,'Value')};
assert(isequal(beforeMotor,afterMotor),'Motor command parameters changed.');

data = get_param(model,'CoderTargetData');
data.STM32CubeMX.ProjectFile = fullfile(taskDir,'hardware','motortest.ioc');
data.Runtime.BuildAction = 'Build'; % explicitly prevent automatic flashing
set_param(model,'CoderTargetData',data);
set_param(model,'InitFcn',sprintf('addpath(''%s'');',strrep(taskDir,'''','''''')));
save_system(model,target);
set_param(model,'SimulationCommand','update');
save_system(model,target);
fprintf('MODEL_SAVED: %s\n',target);
if doBuild
    slbuild(model);
    fprintf('BUILD_ONLY_COMPLETE: no deployment requested.\n');
end
close_system(model,0);
end

function block = oneBlock(model,name)
blocks = find_system(model,'SearchDepth',1,'Type','Block','Name',name);
assert(numel(blocks)==1,'Expected exactly one block named %s.',name);
block = blocks{1};
end

function name = blockName(block)
name = strrep(get_param(block,'Name'),'/','//');
end
