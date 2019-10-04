function [ret]=c_statepids(command,arg1)
global defs statepids

switch (command)
  case defs.name
    ret='pids';
  case defs.init
    if (nargin==2)
      statepids.cnum=arg1;
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
global statepids
statepids.fig=figure('Visible','off','NumberTitle','off','MenuBar','none',...
            'CloseRequestFcn',['togglecomponents(' statepids.cnum ',0);'],...
            'DoubleBuffer','on','Name','pids','Position',[100 100 900 700]);

function start
global statepids conf
figure(statepids.fig);
linedata=double(zeros(1,300));
set(statepids.fig,'Visible','on');
ypos=0.05; ystep=0.09; yskip=0.03;
[pl(9),il(9),dl(9)]=addjoint(0.05,ypos,'','Knee',linedata);
[pl(12),il(12),dl(12)]=addjoint(0.55,ypos,'','Knee',linedata);
ypos=ypos+ystep;
[pl(8),il(8),dl(8)]=addjoint(0.05,ypos,'','Elevator',linedata);
[pl(11),il(11),dl(11)]=addjoint(0.55,ypos,'','Elevator',linedata);
ypos=ypos+ystep;
[pl(7),il(7),dl(7)]=addjoint(0.05,ypos,'Left Back','Rotator',linedata);
[pl(10),il(10),dl(10)]=addjoint(0.55,ypos,'Right Back','Rotator',linedata);
ypos=ypos+ystep+yskip;

[pl(3),il(3),dl(3)]=addjoint(0.05,ypos,'','Knee',linedata);
[pl(6),il(6),dl(6)]=addjoint(0.55,ypos,'','Knee',linedata);
ypos=ypos+ystep;
[pl(2),il(2),dl(2)]=addjoint(0.05,ypos,'','Elevator',linedata);
[pl(5),il(5),dl(5)]=addjoint(0.55,ypos,'','Elevator',linedata);
ypos=ypos+ystep;
[pl(1),il(1),dl(1)]=addjoint(0.05,ypos,'Left Front','Rotator',linedata);
[pl(4),il(4),dl(4)]=addjoint(0.55,ypos,'Right Front','Rotator',linedata);
ypos=ypos+ystep+yskip;

[pl(15),il(15),dl(15)]=addjoint(0.05,ypos,'','Roll',linedata);
[pl(18),il(18),dl(18)]=addjoint(0.55,ypos,'','Jaw',linedata);
ypos=ypos+ystep;
[pl(14),il(14),dl(14)]=addjoint(0.05,ypos,'','Pan',linedata);
[pl(17),il(17),dl(17)]=addjoint(0.55,ypos,'','Tail Pan',linedata);
ypos=ypos+ystep;
[pl(13),il(13),dl(13)]=addjoint(0.05,ypos,'Head','Tilt',linedata);
[pl(16),il(16),dl(16)]=addjoint(0.55,ypos,'Misc','Tail Tilt',linedata);
drawnow;
statepids.lines_p=pl;
statepids.lines_i=il;
statepids.lines_d=dl;
statepids.obj=WorldStatePIDsListener(conf.ip,conf.port_statepids);

function iter
global statepids
if (statepids.obj.isConnected && statepids.obj.hasData)
  data=statepids.obj.getData;
  ps=double(data.P);
  is=double(data.I);
  ds=double(data.D);
  for i=1:18
    ydata=get(statepids.lines_p(i),'YData');
    set(statepids.lines_p(i),'YData',[ydata(2:300) ps(i)]);
    ydata=get(statepids.lines_i(i),'YData');
    set(statepids.lines_i(i),'YData',[ydata(2:300) is(i)]);
    ydata=get(statepids.lines_d(i),'YData');
    set(statepids.lines_d(i),'YData',[ydata(2:300) ds(i)]);
  end
end

function stop
global statepids
statepids.obj.close;
set(statepids.fig,'Visible','off');


function destroy
global statepids
delete(statepids.fig);

function [pl, il, dl]=addjoint(xpos,ypos,name1,name2,initplot)
p=subplot('Position',[xpos ypos 0.4 0.075]);
set(p,'XTick',[]);
ylabel(name2);
title(name1);
hold on
pl=plot(initplot,'Color','b');
il=plot(initplot,'Color','g');
dl=plot(initplot,'Color','r');
