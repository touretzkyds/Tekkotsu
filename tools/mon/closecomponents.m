function closecomponents
global comps defs

for i=1:comps.num
  if (comps.state(i)~=defs.state_stop)
    feval(comps.handles(i),defs.stop);
  end
  feval(comps.handles(i),defs.destroy);
end

