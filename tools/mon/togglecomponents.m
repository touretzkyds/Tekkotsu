function togglecomponents(n,status)
global comps defs

if (nargin==2)
  set(comps.button(n),'Value',status);
else
  status=get(comps.button(n),'Value');
end

if (status)
  feval(comps.handles(n),defs.start); 
  comps.state(n)=defs.state_run;
else
  feval(comps.handles(n),defs.stop); 
  comps.state(n)=defs.state_stop;
end
