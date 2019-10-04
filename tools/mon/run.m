% This file is a script to make handles visible in the matlab workspace

if (exist('aibomon_running','var') && aibomon_running)
  close;
end

clear
global defs conf comps mon aibomon_running

addpath '.';

aibomon_running=1;
defines;
config;
loadcomponents;

launcher;

% Instead of looping continuously, run the update loop periodically.
% This allows the user to play around with stuff.
timer=timer('Period',0.01,'TimerFcn','itercomponents', ...
            'ExecutionMode','fixedRate');
start(timer);
