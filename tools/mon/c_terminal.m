function [ret]=c_terminal(command,arg1)
global defs terminal

switch (command)
  case defs.name
    ret='terminal';
  case defs.init
    if (nargin==2)
      terminal.cnum=arg1;
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

function start
global terminal conf
terminal.obj=Terminal;
terminal.stdoutobj=TextListener(conf.ip,conf.port_stdout);
terminal.stderrobj=TextListener(conf.ip,conf.port_stderr);

function iter
global terminal
if (terminal.stdoutobj.isConnected && terminal.stdoutobj.hasData)
  terminal.obj.write(terminal.stdoutobj.getData);
end
if (terminal.stderrobj.isConnected && terminal.stderrobj.hasData)
  terminal.obj.write(terminal.stderrobj.getData);
end
if (terminal.obj.hasData)
  terminal.stdoutobj.write(terminal.obj.getData);
end
if (terminal.obj.wasClosed)
  togglecomponents(str2num(terminal.cnum),0) 
end

function stop
global terminal
terminal.obj.close;
terminal.stdoutobj.close;
terminal.stderrobj.close;

function destroy
