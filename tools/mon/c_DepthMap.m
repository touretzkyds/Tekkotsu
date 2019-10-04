function [ret]=c_DepthMap(command,arg1)
global defs SDM
switch (command)
  case defs.name
    ret='depth map';
  case defs.init
    if (nargin==2)
      SDM.cnum = arg1;
      init;
    else
      disp('error: init ignored, no handle specified');
    end
  case defs.start
    start;
  case defs.iter
    iter;
  case defs.stop
    stop;
  case defs.destroy
    destroy;
  otherwise
    disp('warning: component called with invalid command');
end


function init
global SDM
SDM.fig = figure('Visible','off','NumberTitle','off',...
		 'CloseRequestFcn',['togglecomponents(' SDM.cnum ',0);'],...
		 'DoubleBuffer','on','Name','Spherical depth map');


function start
global SDM conf
  %% create figure
figure(SDM.fig);
set(SDM.fig, 'Visible', 'on');
SDM.obj = WM2DMListener(conf.ip, conf.port_WM2DM);
SDM.params = WorldModel2Conf;
  %% load colormap
load (conf.cmap_visionseg);
SDM.cmap=cmap;
  %% create and plot unit vectors
[SDM.unit_x, SDM.unit_y, SDM.unit_z] = computeDMUnitVectors;
SDM.plotHandle = surf(SDM.unit_x, SDM.unit_y, SDM.unit_z);
SDM.axisHandle = get(SDM.plotHandle,'Parent');
set(SDM.fig,'Renderer','opengl');
colormap(SDM.axisHandle,cmap);
set(SDM.plotHandle,'CDataMapping','direct');
set(SDM.plotHandle,'AlphaData',...
    ones(SDM.params.ALM_DM_V_SIZE, SDM.params.ALM_DM_H_SIZE));
set(SDM.plotHandle,'AlphaDataMapping','none');
%Uncomment on OpenGL enabled machines
set(SDM.plotHandle,'FaceAlpha','interp');
axis equal;


function iter
global SDM
if(SDM.obj.isConnected && SDM.obj.hasData)
  DMdata = SDM.obj.getData;
  Depths = double(DMdata.DM_depth);
  Depths(find(Depths>800)) = NaN;
  Depths = reshape(Depths, SDM.params.ALM_DM_H_SIZE, SDM.params.ALM_DM_V_SIZE)';
  X = SDM.unit_x .* Depths;
  Y = SDM.unit_y .* Depths;
  Z = SDM.unit_z .* Depths;
  Colors = double(DMdata.DM_color) + 1;
  Colors = reshape(Colors, SDM.params.ALM_DM_H_SIZE, SDM.params.ALM_DM_V_SIZE)';
  Alphas = double(DMdata.DM_confidence) .* 0.5 + 0.5;
  Alphas = reshape(Alphas, SDM.params.ALM_DM_H_SIZE, SDM.params.ALM_DM_V_SIZE)';
  set(SDM.plotHandle,'Xdata',X);
  set(SDM.plotHandle,'Ydata',Y);
  set(SDM.plotHandle,'Zdata',Z);
  set(SDM.plotHandle,'Cdata',Colors);
  set(SDM.plotHandle,'AlphaData',Alphas);
  axis equal;
end


function stop
global SDM
SDM.obj.close;
set(SDM.fig, 'Visible', 'off');


function destroy
global SDM
delete(SDM.fig);


function [unit_x,unit_y,unit_z]=computeDMUnitVectors
% Computes the unit vectors for each pixel in the DM
global SDM
azimuth  = linspace(SDM.params.ALM_DM_LEFT, SDM.params.ALM_DM_RIGHT,...
		    SDM.params.ALM_DM_H_SIZE);
altitude = linspace(SDM.params.ALM_DM_TOP, SDM.params.ALM_DM_BOTTOM,...
		    SDM.params.ALM_DM_V_SIZE);

[azimuth, altitude] = meshgrid(azimuth, altitude);

cosnalt = cos(altitude);
sinnalt = sin(altitude);
cosnaz  = cos(azimuth);
sinnaz  = sin(azimuth);

unit_x = (cosnaz .* cosnalt);
unit_y = (sinnaz .* cosnalt);
unit_z = sinnalt;
