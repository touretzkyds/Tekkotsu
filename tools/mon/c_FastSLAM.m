function [ret]=c_FastSLAM(command,arg1)
global defs FS
switch (command)
  case defs.name
    ret='FastSLAM';
  case defs.init
    if (nargin==2)
      FS.cnum = arg1;
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
global FS
FS.fig = figure('Visible','off','NumberTitle','off',...
		'CloseRequestFcn',['togglecomponents(' FS.cnum ',0);'],...
		'DoubleBuffer','on','Name','FastSLAM map');
FS.axis = axis;


function start
global FS conf
  %% create figure
figure(FS.fig);
set(FS.fig, 'Visible', 'on');
FS.obj = WM2FSListener(conf.ip, conf.port_WM2FS);
FS.params = WorldModel2Conf;


function iter
global FS
if(FS.obj.isConnected && FS.obj.hasData)
    % init new plot
  axis(FS.axis);
  cla reset; axis equal; hold on;
    % get and process data
  FSdata = FS.obj.getData;
  particles_X = double(FSdata.FS_particlesPos(:,1));
  particles_Y = double(FSdata.FS_particlesPos(:,2));
  landmarks_X = double(FSdata.FS_landmarkPos(:,1));
  landmarks_Y = double(FSdata.FS_landmarkPos(:,2));
  landmarks_X(find(landmarks_X > 100000000)) = NaN;
  covariances = double(FSdata.FS_covariance);
    % plot it
  plot(particles_X, particles_Y, 'y+');
  plot(landmarks_X, landmarks_Y, 'b*');
  plot(FSdata.FS_x, FSdata.FS_y, 'r+');
  plot_line(FSdata.FS_x, FSdata.FS_y, 60, FSdata.FS_theta);
  for lm = find(~isnan(landmarks_X)')
    plot_ellipse(reshape(covariances(lm,:),2,2),...
		 landmarks_X(lm), landmarks_Y(lm));
  end
end


function stop
global FS
FS.obj.close;
set(FS.fig, 'Visible', 'off');


function destroy
global FS
delete(FS.fig);


function plot_ellipse(S, x, y)
% Used to plot covariance matrix ellipses for the unit test code
% make a unit circle (parameterized by r)
r = 0:.1:2*pi;
temp_x = cos(r);
temp_y = sin(r);
% do chevrolet magic:
    A = chol(S);
% apply that transformation to the circle
    ellipse = A*[temp_x; temp_y];
% wah-lah, we have an ellipse, so let's plot it:
plot(ellipse(1,:) + x, ellipse(2,:) + y, 'c');


function plot_line(x,y,dist,theta)
% Used to plot lines for the unit test code
far_x = cos(theta) * dist + x;
far_y = sin(theta) * dist + y;
line([x far_x], [y far_y]);
