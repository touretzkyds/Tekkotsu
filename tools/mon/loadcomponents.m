function loadcomponents
global comps defs

list=dir('c_*.m');
comps.num=size(list,1);
for i=1:comps.num
  name=strtok(list(i).name,'.');
  comps.handles(i)=str2func(name);
  comps.state(i)=0;
  feval(comps.handles(i),defs.init,num2str(i));
end

