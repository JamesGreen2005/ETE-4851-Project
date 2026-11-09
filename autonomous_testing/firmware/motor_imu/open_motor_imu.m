% Open the prepared model. This script DOES NOT build or flash the board.
taskDir = fileparts(mfilename('fullpath'));
addpath(taskDir);
taskModelFile = fullfile(taskDir,'Motor_IMU_Test.slx');
load_system(taskModelFile);
taskTarget = get_param('Motor_IMU_Test','CoderTargetData');
taskTarget.Runtime.BuildAction = 'Build, load and run';
set_param('Motor_IMU_Test','CoderTargetData',taskTarget);
open_system('Motor_IMU_Test');
fprintf('Motor_IMU_Test opened. Keep motor power OFF for the first upload.\n');
fprintf('Click Hardware > Run on board > Build, Deploy & Start when ready.\n');
fprintf('The existing 50%% motor command remains in this model.\n');
fprintf('After deployment, connect ST-LINK USB to the Pi and open its serial terminal at 115200 baud.\n');
