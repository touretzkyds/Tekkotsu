function [ret]=c_statejoints(command,arg1)
global defs statejoints

switch (command)
  case defs.name
    ret='joints';
  case defs.init
    if (nargin==2)
      statejoints.cnum=arg1;
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
global statejoints
statejoints.fig=figure('Visible','off','NumberTitle','off','MenuBar','none',...
            'CloseRequestFcn',['togglecomponents(' statejoints.cnum ',0);'],...
            'DoubleBuffer','on','Name','joints','Position',[100 100 900 700]);

function start
global statejoints conf
figure(statejoints.fig);
linedata=double(zeros(1,300));
set(statejoints.fig,'Visible','on');
ypos=0.05; ystep=0.09; yskip=0.03;
[pos(9),duty(9)]=addjoint(0.05,ypos,'','Knee',linedata);
[pos(12),duty(12)]=addjoint(0.55,ypos,'','Knee',linedata);
ypos=ypos+ystep;
[pos(8),duty(8)]=addjoint(0.05,ypos,'','Elevator',linedata);
[pos(11),duty(11)]=addjoint(0.55,ypos,'','Elevator',linedata);
ypos=ypos+ystep;
[pos(7),duty(7)]=addjoint(0.05,ypos,'Left Back','Rotator',linedata);
[pos(10),duty(10)]=addjoint(0.55,ypos,'Right Back','Rotator',linedata);
ypos=ypos+ystep+yskip;

[pos(3),duty(3)]=addjoint(0.05,ypos,'','Knee',linedata);
[pos(6),duty(6)]=addjoint(0.55,ypos,'','Knee',linedata);
ypos=ypos+ystep;
[pos(2),duty(2)]=addjoint(0.05,ypos,'','Elevator',linedata);
[pos(5),duty(5)]=addjoint(0.55,ypos,'','Elevator',linedata);
ypos=ypos+ystep;
[pos(1),duty(1)]=addjoint(0.05,ypos,'Left Front','Rotator',linedata);
[pos(4),duty(4)]=addjoint(0.55,ypos,'Right Front','Rotator',linedata);
ypos=ypos+ystep+yskip;

[pos(15),duty(15)]=addjoint(0.05,ypos,'','Roll',linedata);
[pos(18),duty(18)]=addjoint(0.55,ypos,'','Jaw',linedata);
ypos=ypos+ystep;
[pos(14),duty(14)]=addjoint(0.05,ypos,'','Pan',linedata);
[pos(17),duty(17)]=addjoint(0.55,ypos,'','Tail Pan',linedata);
ypos=ypos+ystep;
[pos(13),duty(13)]=addjoint(0.05,ypos,'Head','Tilt',linedata);
[pos(16),duty(16)]=addjoint(0.55,ypos,'Misc','Tail Tilt',linedata);
ypos=ypos+ystep;
[but(1),but(2),but(3),but(4)]=addbuttons(0.05,ypos,'Buttons','Buttons 1-4',linedata);
[but(5),but(6),but(7),but(8)]=addbuttons(0.55,ypos,'Buttons','Buttons 5-8',linedata);
drawnow;
statejoints.lines_pos=pos;
statejoints.lines_duty=duty;
statejoints.lines_but=but;
statejoints.obj=org.tekkotsu.mon.WorldStateJointsListener(conf.ip,conf.port_statejoints);

function iter
global statejoints
if (statejoints.obj.isConnected && statejoints.obj.hasData)
  data=statejoints.obj.getData;
  positions=double(data.positions);
  duties=double(data.duties);
  buttons=double(data.buttons);
  for i=1:18
    ydata=get(statejoints.lines_pos(i),'YData');
    set(statejoints.lines_pos(i),'YData',[ydata(2:300) positions(i)]);
    ydata=get(statejoints.lines_duty(i),'YData');
    set(statejoints.lines_duty(i),'YData',[ydata(2:300) duties(i)]);
  end
  for i=1:8
    ydata=get(statejoints.lines_but(i),'YData');
    set(statejoints.lines_but(i),'YData',[ydata(2:300) (buttons(i)/2+mod(i-1,4))]);
  end
end

function stop
global statejoints
statejoints.obj.close;
set(statejoints.fig,'Visible','off');


function destroy
global statejoints
delete(statejoints.fig);

function [pos, duty]=addjoint(xpos,ypos,name1,name2,initplot)
p=subplot('Position',[xpos ypos 0.4 0.075]);
set(p,'XTick',[]);
set(p,'YTick',-pi/2:pi/2:pi/2);
set(p,'YTickLabel',{'-pi/2','0','pi/2'});
set(p,'YLim',[-pi/2 pi/2]);
ylabel(name2);
title(name1);
hold on
pos=plot(initplot,'Color','b');
duty=plot(initplot,'Color','r');

function [but1,but2,but3,but4]=addbuttons(xpos,ypos,name1,name2,initplot)
p=subplot('Position',[xpos ypos 0.4 0.075]);
set(p,'XTick',[]);
set(p,'YTick',-1:2:1);
set(p,'YTickLabel',{'1','2','3','4'});
set(p,'YLim',[0 4.5]);
ylabel(name2);
title(name1);
hold on
but1=plot(initplot,'Color','r');
but2=plot(initplot,'Color','g');
but3=plot(initplot,'Color','b');
but4=plot(initplot,'Color','m');

