function build_vehicle(doBuild)
% Create a separate model and compile only. Never upload to hardware.
if nargin==0, doBuild=true; end
taskDir=fileparts(mfilename('fullpath'));
imuDir=fullfile(fileparts(taskDir),'motor_imu');
addpath(taskDir,imuDir);
cd(taskDir);
test_vehicle_protocol;
model='Vehicle_Drive';
if bdIsLoaded(model), error('Save and close Vehicle_Drive first.'); end
load_system(fullfile(fileparts(taskDir),'pi_motor_imu','Motor_Pi_IMU.slx'));
save_system('Motor_Pi_IMU',fullfile(taskDir,[model '.slx']));
root=sfroot;
chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Pi Motor Command']);
chart.Script=sprintf(['function [duty,enable,armed,commandError,servoPulse] = command(data,length,status)\n' ...
 '[duty,enable,armed,commandError,servoPulse] = vehicle_command(data,length,status);\nend\n']);
% Split independent signed duties; route motor 1 polarity to separate PWM legs.
delete_line(model,'Pi Motor Command/1','PWM Output/1');
add_block('simulink/Signal Routing/Demux',[model '/Two Motor Duties'],'Outputs','2','Position',[460 25 465 85]);
add_line(model,'Pi Motor Command/1','Two Motor Duties/1','autorouting','on');
delete_line(model,'Constant1/1','PWM Output/2');
delete_block([model '/Constant1']);
split=[model '/Motor 1 Direction'];
add_block('simulink/User-Defined Functions/MATLAB Function',split,'Position',[470 100 640 150]);
chart=root.find('-isa','Stateflow.EMChart','Path',split);
chart.Script=sprintf('function [forward,reverse]=split(duty)\nforward=max(0,duty); reverse=max(0,-duty);\nend\n');
add_line(model,'Two Motor Duties/1','Motor 1 Direction/1','autorouting','on');
add_line(model,'Motor 1 Direction/1','PWM Output/1','autorouting','on');
add_line(model,'Motor 1 Direction/2','PWM Output/2','autorouting','on');
set_param([model '/PWM Output'],'SampleTime','0.001');
block=[model '/Motor 2 and Encoders'];
add_block('simulink/User-Defined Functions/MATLAB Function',block,'Position',[520 245 700 315]);
chart=root.find('-isa','Stateflow.EMChart','Path',block);
chart.Script=sprintf(['function [e1,e2,s1,s2,period] = io(duty,enable,servoPulse)\n%%#codegen\n' ...
 'e1=uint16(0); e2=uint16(0); s1=uint16(0); s2=uint16(0); period=uint16(0);\n' ...
 'if ~coder.target(''MATLAB'') && ~coder.target(''Sfun'')\n' ...
 'coder.cinclude(''vehicle_io.h'');\n' ...
 'coder.ceval(''vehicle_io'',double(duty),uint8(enable),coder.wref(e1),coder.wref(e2));\n' ...
 'coder.ceval(''vehicle_servo_io'',double(servoPulse(1)),double(servoPulse(2)));\n' ...
 'coder.ceval(''vehicle_servo_status'',coder.wref(s1),coder.wref(s2),coder.wref(period));\n' ...
 'end\nend\n']);
add_line(model,'Two Motor Duties/2','Motor 2 and Encoders/1','autorouting','on');
add_line(model,'Pi Motor Command/2','Motor 2 and Encoders/2','autorouting','on');
add_line(model,'Pi Motor Command/5','Motor 2 and Encoders/3','autorouting','on');
chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Readable Angles']);
chart.Script=sprintf(['function text = formatAngles(h,r,p,cal,valid,err,armed,duty,cmdErr,e1,e2,s1,s2,period)\n' ...
 'text=vehicle_text(h,r,p,cal,valid,err,armed,duty,cmdErr,e1,e2,s1,s2,period);\nend\n']);
add_line(model,'Motor 2 and Encoders/1','Readable Angles/10','autorouting','on');
add_line(model,'Motor 2 and Encoders/2','Readable Angles/11','autorouting','on');
add_line(model,'Motor 2 and Encoders/3','Readable Angles/12','autorouting','on');
add_line(model,'Motor 2 and Encoders/4','Readable Angles/13','autorouting','on');
add_line(model,'Motor 2 and Encoders/5','Readable Angles/14','autorouting','on');
data=get_param(model,'CoderTargetData');
% Reuse unchanged CubeMX project: extra timer/GPIO ownership is in vehicle_io.c.
data.STM32CubeMX.ProjectFile=fullfile(fileparts(taskDir),'pi_motor_imu','hardware','motortest.ioc');
data.Runtime.BuildAction='Build';
% Readback telemetry is 209 bytes; the default 128-byte ring cannot hold it.
data.USART.USART3TransmitBufferLength='256';
set_param(model,'CoderTargetData',data);
set_param(model,'CustomInclude','.','CustomSource','vehicle_io.c current_ina219.c');
set_param(model,'InitFcn',sprintf('addpath(''%s'',''%s'');',strrep(taskDir,'''',''''''),strrep(imuDir,'''','''''')));
save_system(model);
set_param(model,'SimulationCommand','update');
save_system(model);
fprintf('MODEL_SAVED: %s\n',fullfile(taskDir,[model '.slx']));
if doBuild
    slbuild(model);
    resources=fileread(fullfile(taskDir,[model '_ert_rtw'],'MW_target_hardware_resources.h'));
    token=regexp(resources,'MW_USART_USART3TRANSMITBUFFERLENGTH\s+(\d+)','tokens','once');
    assert(~isempty(token) && str2double(token{1})>=256,'Host UART TX buffer was not enlarged.');
    fprintf('UART_BUFFER_VERIFIED: 209-byte telemetry fits 256-byte host TX buffer.\n');
    fprintf('BUILD_ONLY_COMPLETE: no firmware uploaded.\n');
end
close_system(model,0);
end
