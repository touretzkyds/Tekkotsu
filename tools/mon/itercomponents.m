function itercomponents
global comps defs

for i=1:comps.num
  if (comps.state(i)==defs.state_run)
    feval(comps.handles(i),defs.iter);
  end
end

