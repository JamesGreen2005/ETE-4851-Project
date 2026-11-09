% Opens the new model, but does NOT flash or move the motor.
taskDir=fileparts(mfilename('fullpath'));
imuDir=fullfile(fileparts(taskDir),'motor_imu');
addpath(taskDir,imuDir);
load_system(fullfile(taskDir,'Motor_Pi_IMU.slx'));
taskData=get_param('Motor_Pi_IMU','CoderTargetData');
taskData.Runtime.BuildAction='Build, load and run';
set_param('Motor_Pi_IMU','CoderTargetData',taskData);
open_system('Motor_Pi_IMU');
fprintf('Motor_Pi_IMU: starts DISARMED; no automatic 50%% startup.\n');
fprintf('Keep motor power OFF during first upload.\n');
fprintf('Upload using Hardware > Run on board > Deploy > Build, Deploy & Start.\n');
fprintf('Then connect USB to Pi and run pi_motor_control.py.\n');
