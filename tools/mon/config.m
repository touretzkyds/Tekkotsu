function config
global conf

%eventually parse single config file

conf.ip='172.16.1.2'; % Aibo's IP address

conf.port_stdout=10001;
conf.port_stderr=10002;
conf.port_visionraw=10011;
conf.port_visionseg=10012;
conf.port_statejoints=10031;
conf.port_statepids=10032;
conf.port_WM2DM=10041;
conf.port_WM2HM=10042;
conf.port_WM2GM=10043;
conf.port_WM2FS=10044;
conf.port_mecha=10050;

conf.cmap_visionseg='colormap_rle_NSHA431.mat';

% Interval at which mecha code automatically sends current motion parameters
% to the robot.
% Right now it's set to one second. You could set it for days.
conf.mecha_autosend_interval = datenum('00:00:01');
