function launcher
global comps defs mon

bw=100; bh=25; bx=0; by=comps.num*25;

mon.launcher=figure('Unit','pixels','Position',[20 50 100 by]);
clf reset
set(gcf,'CloseRequestFcn','close');
set(mon.launcher,'Name','AiboMon','MenuBar','none','NumberTitle','off');

for i=1:comps.num
  by=by-bh;
  name=feval(comps.handles(i),defs.name);
  comps.button(i) = ...
   uicontrol('Style','CheckBox','Position',[bx by bw bh],'Value',0,...
             'String',name,'FontSize',10, ...
             'CallBack',['togglecomponents(' num2str(i) ');']);
end

