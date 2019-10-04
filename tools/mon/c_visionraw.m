function [ret]=c_visionraw(command,arg1)
global defs visionraw

switch (command)
  case defs.name
    ret='vision raw';
  case defs.init
    if (nargin==2)
      visionraw.cnum=arg1;
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
global visionraw
visionraw.fig=figure('Visible','off','NumberTitle','off','MenuBar','none',...
            'CloseRequestFcn',['togglecomponents(' visionraw.cnum ',0);'],...
            'DoubleBuffer','on','Name','vision raw',...
            'Position',[400 400 144 176]);

function start
global visionraw conf
visionraw.imdata=double(zeros(72,88,3));
figure(visionraw.fig);
set(gca,'Position',[0 0.1 1 1])
visionraw.im=image(visionraw.imdata);
set(visionraw.fig,'Visible','on');
visionraw.obj=VisionRawListener(conf.ip,conf.port_visionraw);

function iter
global visionraw
if (visionraw.obj.isConnected && visionraw.obj.hasData)
  data=visionraw.obj.getData;
  % extract y,u,v channel data
  imdata(:,:,1)=reshape(data(1:3:size(data)),[88 72])';
  imdata(:,:,2)=reshape(data(2:3:size(data)),[88 72])';
  imdata(:,:,3)=reshape(data(3:3:size(data)),[88 72])';
  % make unsigned
  imdata=double(imdata)+(imdata<0)*256;
  % convert to rgb
  visionraw.imdata=min(1,max(yuv2rgb(imdata)/255,0));
  set(visionraw.im,'cdata',visionraw.imdata);
end

function stop
global visionraw
visionraw.obj.close;
set(visionraw.fig,'Visible','off');


function destroy
global visionraw
delete(visionraw.fig);
