function [ret]=c_HeightMap(command,arg1)
global defs HHM
switch (command)
  case defs.name
    ret='height map';
  case defs.init
    if (nargin==2)
      HHM.cnum = arg1;
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
global HHM
HHM.fig = figure('Visible','off','NumberTitle','off',...
		 'CloseRequestFcn',['togglecomponents(' HHM.cnum ',0);'],...
		 'DoubleBuffer','on','Name','Horizontal height map');


function start
global HHM conf
  %% create figure
figure(HHM.fig);
set(HHM.fig, 'Visible', 'on');
HHM.obj = WM2HMListener(conf.ip, conf.port_WM2HM);
HHM.params = WorldModel2Conf;
  %% the radius-based map size is confusing, so we make a note of the real
  %% edge size of the matrix. see note in WorldModel2Conf.java
HHM.ALM_HM_TRUESIZE = 2*HHM.params.ALM_HM_SIZE;
  %% load colormap
load (conf.cmap_visionseg);
HHM.cmap=cmap;
  %% append "dark" cmap to cmap for showing untraversable areas, but also
  %% keep track of original cmap size
HHM.cmapsize=length(cmap);
cmap=[cmap; cmap.*0.3];
  %% create x and y arrays--eventually they'll be used to make a surface
  %% then plot initial surface
[HHM.X, HHM.Y] = computeHMXY;
HHM.plotHandle = surf(HHM.X, HHM.Y, zeros(HHM.ALM_HM_TRUESIZE));
HHM.axisHandle = get(HHM.plotHandle,'Parent');
set(HHM.fig,'Renderer','opengl');
colormap(HHM.axisHandle,cmap);
set(HHM.plotHandle,'CDataMapping','direct');
set(HHM.plotHandle,'AlphaData',ones(HHM.ALM_HM_TRUESIZE));
set(HHM.plotHandle,'AlphaDataMapping','none');
%Uncomment on OpenGL enabled machines
set(HHM.plotHandle,'FaceAlpha','interp');
axis equal;


function iter
global HHM
if(HHM.obj.isConnected && HHM.obj.hasData)
  HMdata = HHM.obj.getData;
  Heights = double(HMdata.HM_height);
  Heights = reshape(Heights, HHM.ALM_HM_TRUESIZE, HHM.ALM_HM_TRUESIZE);
  Colors = double(HMdata.HM_color) + 1;
  Colors = reshape(Colors, HHM.ALM_HM_TRUESIZE, HHM.ALM_HM_TRUESIZE);
  Alphas = double(HMdata.HM_confidence) .* 0.5 + 0.5;
  Alphas = reshape(Alphas, HHM.ALM_HM_TRUESIZE, HHM.ALM_HM_TRUESIZE);
    %% Make untraversable parts of map dark
  Untravs = double(HMdata.HM_trav) < 0.5;
  Untravs = reshape(Untravs, HHM.ALM_HM_TRUESIZE, HHM.ALM_HM_TRUESIZE);
  Colors = Colors + Untravs .* HHM.cmapsize;
    %% Plot!
  set(HHM.plotHandle,'Zdata',Heights);
  set(HHM.plotHandle,'Cdata',Colors);
  set(HHM.plotHandle,'AlphaData',Alphas);
  axis equal;
end


function stop
global HHM
HHM.obj.close;
set(HHM.fig, 'Visible', 'off');


function destroy
global HHM
delete(HHM.fig);


function [X,Y]=computeHMXY
% generate X and Y matrices for HHM surf plotting
global HHM
index = linspace(-HHM.params.ALM_HM_RADIUS,...
		  HHM.params.ALM_HM_RADIUS, HHM.ALM_HM_TRUESIZE);
[X,Y] = meshgrid(index);
knockout = find((X.*X + Y.*Y) > HHM.params.ALM_HM_RADIUS^2);
X(knockout) = NaN;
Y(knockout) = NaN;
