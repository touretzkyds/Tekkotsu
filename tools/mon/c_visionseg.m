function [ret]=c_visionseg(command,arg1)
global defs visionseg

switch (command)
  case defs.name
    ret='vision seg';
  case defs.init
    if (nargin==2)
      visionseg.cnum=arg1;
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
global visionseg
visionseg.fig=figure('Visible','off','NumberTitle','off','MenuBar','none',...
            'CloseRequestFcn',['togglecomponents(' visionseg.cnum ',0);'],...
            'DoubleBuffer','on','Name','vision seg');

function start
global visionseg conf
load (conf.cmap_visionseg);
visionseg.cmap=cmap;
visionseg.imdata=uint8(ones(144,176));
figure(visionseg.fig);
visionseg.im=imshow(visionseg.imdata,visionseg.cmap);
set(visionseg.fig,'Visible','on');
visionseg.obj=VisionRleListener(conf.ip,conf.port_visionseg);

function iter
global visionseg
if (visionseg.obj.isConnected && visionseg.obj.hasData)
  data=visionseg.obj.getData;
  visionseg.imdata=reshape(data,[176 144])';
  set(visionseg.im,'cdata',visionseg.imdata);
end

function stop
global visionseg
visionseg.obj.close;
set(visionseg.fig,'Visible','off');


function destroy
global visionseg
delete(visionseg.fig);
