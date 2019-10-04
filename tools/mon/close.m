function close
global mon comps defs aibomon_running

stop (timer);
closecomponents;
delete(mon.launcher);
aibomon_running=0;
