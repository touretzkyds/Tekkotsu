function plotlog(logfilein)
     global logfile info jpl plotops legh ploth subp timeslider window aibopos jgraph pgraph posgraph bodyangle updatego legpos legspos samplecount bodypos extra plotlog_inited;
     updatego=0;
     ploth=[];
     subp=[];
     if(isempty(plotops))
       plotops=struct('drawbuttons',1);
     end
     logfile=logfilein;
  ers2xxinfo;
bodyangle=.20944;
samplecount=size(logfile,1);
aibopos=zeros(4,3,samplecount);
legpos=zeros(4,3,samplecount);
jpl=info.JointsPerLeg; %because it's used so much...
  plotlog_inited=1;
  jgraph=figure(1);
  pgraph=figure(2);
  posgraph=figure(3);
  extra=figure(4);

  set(jgraph,'Name','Joint positions');
  set(pgraph,'Name','Aibo Feet Accumulator');
  set(posgraph,'Name','Foot trajectory');
  set(extra,'Name','Extra data');

  figure(jgraph);

clf;

  window=struct('size',logfile(end,1)-logfile(1,1),'start',logfile(1,1),'zoomfactor',1);

  timeslider=uicontrol('style','slider','position',[100,0,100,20],'callback','setslide;');
  zoomin=uicontrol('style','pushbutton','position',[250,0,50,20],'string','Zoom in','callback','zoomin;');
  zoomout=uicontrol('style','pushbutton','position',[350,0,50,20],'string','Zoom out','callback','zoomout;');

  for leg=1:4
    plotleg(leg,getJointAngles(leg),getDuties(leg),getButton(leg));
  end

  legh=legend(info.legJointOrder{:},0);
  posnow=get(legh,'position');
  posnow(1:2)=[0 0];
  set(legh,'position',posnow);
  
  %%%%basic stuff done

bodypos=trackMovement;

figure(pgraph);
clf;
plotpath;
figure(posgraph);
clf;
plotpos;
drawBody(bodypos);

%extra
figure(extra);
%rot=mod(bodypos(:,4),360);
rot=bodypos(:,4);
plot(rot);

%fit graph

updatego=1;
updateAxes; %really just fix slider

function plotleg(leg,position,duties,button)
global logfile info jpl ploth subp plotops window defax hb aibopos pgraph jgraph legpos;
ontime=extractButtonOn(button);
  subp(end+1)=subplot(2,4,leg);
  hb=plot(logfile(:,1),position)';
  title([info.legOrder{leg} ' pos']);
  if(plotops.drawbuttons~=0)
    hold on
    hb=[hb plot(logfile(ontime,1),position(ontime,:),'o')'];
plot(logfile(:,1),button,':k');
  end
  ploth=[ploth hb];
  subp(end+1)=subplot(2,4,leg+4);
  hb=plot(logfile(:,1),duties)';
  title([info.legOrder{leg}, ' duties']);
  if(plotops.drawbuttons~=0)
    hold on
    hb=[hb plot(logfile(ontime,1),duties(ontime,:),'o')'];
  end
  ploth=[ploth hb];
  defax=axis;

  %forward kinematics tracking:
  for step=1:length(button)
    legPoint(step,leg); %precalc legpositions
  end
  altbutton=getButton(mod(leg+2,4)+1);
  for step=2:(length(button))
%    if(button(step)>0 | altbutton(step)>0) %counts feet as "down" if either front or back is on. this turns out to be bad
    if(button(step)>0)
      delta=legpos(leg,:,step)-legpos(leg,:,step-1);
    else
      delta=[0 0 0];
    end
    aibopos(leg,:,step)=aibopos(leg,:,step-1)-delta; %%aibo moves opposite of foot push
  end

%aibopos=legpos; %%% testing hack!

function pos=getJointAngles(leg)
     global logfile info jpl;
leg=leg-1;
    pos=logfile(:,info.LogOffset.positions+leg*jpl:1:info.LogOffset.positions+((leg+1)*jpl-1));

function duties=getDuties(leg)
     global logfile info jpl;
leg=leg-1;
  duties=logfile(:,(leg*jpl+info.LogOffset.duties):1:((leg+1)*jpl+info.LogOffset.duties-1));
     
function buttons=getButton(leg)
     global logfile info jpl;
leg=leg-1;
  buttons=logfile(:,info.LogOffset.buttons+leg);

function onidx=extractButtonOn(button)
   onidx=find(button>0);

function onidx=findButtonOn(leg)
   global logfile info jpl;
   button=getButton(leg);
   onidx=find(button>0);

%%%%%%%%%%%%%
%% gui stuff
%%%%%%%%%%%%%



%%%%%%%%%
%% Forward kinematics stuff
%%%%%%%%%

function R=rotate(x,y,z,theta)
  c=cos(theta);
  s=sin(theta);
  t=1-cos(theta);
  R=[t*x^2+c, t*x*y-s*z, t*x*z+s*y, 0; ...
     t*x*y+s*z, t*y^2+c, t*y*z-s*x, 0; ...
     t*x*z-s*y, t*y*z+s*x, t*z^2+c, 0; ...
     0, 0, 0, 1 ];

function T=translate(t)
  x=t(1);
 y=t(2);
z=t(3);
  T=[1, 0, 0, x ; ...
     0, 1, 0, y ; ...
     0, 0, 1, z ; ...
     0, 0, 0, 1 ];

function dpt=diffPoints(t1,t2,leg)
  global logfile info jpl legpos;
  pos=getJointAngles(leg);
  buttons=getButton(leg);

  limblen=info.limblen(leg,:,:);
  pos1=pos(t1,:);
  pos2=pos(t2,:);
  pt=[0 0 0 1]; %homogenous point baby
  M=eye(4);
% my coordinate system: y+ = forward, x+ = right, z+ = up
  M=M*translate( -limblen(:,:,1) ); %initial position separated from body
  M=M*rotate(1,0,0,pos1(1)); %rotator - around x 
  M=M*rotate(0,1,0,pos1(2)); %elevator - around y
  M=M*translate( -limblen(:,:,2) );
  M=M*rotate(1,0,0,pos1(3)); %knee - around x
  M=M*translate( -limblen(:,:,3) );
  groundpt1=M*pt';

%	   legpos(leg,:,t1)=groundpt1(1:3);

  M=eye(4);
  M=M*translate( -limblen(:,:,1) ); %initial position separated from body
  M=M*rotate(1,0,0,pos1(1)); %rotator - around x 
  M=M*rotate(0,1,0,pos1(2)); %elevator - around y
  M=M*translate( -limblen(:,:,2) );
  M=M*rotate(1,0,0,pos1(3)); %knee - around x
  M=M*translate( -limblen(:,:,3) );
  groundpt2=M*pt';

%	   legpos(leg,:,t1)=groundpt2(1:3);

%  if(buttons(t1)<1) % foot starting in air
%    dpt=[0,0,0]; %no movement
%    return; 
%else
  dpt=groundpt1-groundpt2;
%end

;  toerotateX=(pos1(3)-pos1(1))-(pos2(3)-pos2(1));
;	   toerotateY=(pos1(2)-pos2(2));
;  M=M*rotate(1,0,0,toerotateX); %assuming shoulder stays even height, and flat ground, parallel lines are your friend. this is basically change in toe-tilt
;%  M=M*rotate(0,1,0,toerotateY);
;  M=M*translate( limblen(:,:,3) );
;  M=M*rotate(1,0,0,pos2(3)); %knee - around x
;  M=M*translate( limblen(:,:,2) );
;  M=M*rotate(0,1,0,pos2(2)); %elevator - around y
;  M=M*rotate(1,0,0,pos2(1)); %rotator - around x 
;  M=M*translate( limblen(:,:,1) );
;  bodypt2=M*pt';
;  dpt=bodypt2(1:3)';


function footpoint=legPoint(t1,leg)
global logfile info jpl legpos;
pos=getJointAngles(leg);
buttons=getButton(leg);

limblen=info.limblen(leg,:,:);
pos1=pos(t1,:);
pt=[0 0 0 1]; %homogenous point baby
M=eye(4);
if(leg>2) %if it's a back foot
      pos1(1)=-pos1(1); %flip it's y rotation
      pos1(3)=-pos1(3); %flip it's y rotation
end
% my coordinate system: x+ = forward, y+ = right, z+ = up
M=M*translate( limblen(:,1,:) ); %initial position separated from body
M=M*rotate(0,1,0,-pos1(1)); %rotator - around y (but +pos(1) means back on dog)
if(mod(leg,2)==0) %if it's a right foot...
      pos1(2)=-pos1(2); %flip it's x rotation
end
M=M*rotate(1,0,0,pos1(2)); %elevator - around x
M=M*translate( limblen(:,2,:) );
M=M*rotate(0,1,0,-pos1(3)); %knee - around y
M=M*translate( limblen(:,3,:) );
footpoint=M*pt';
footpoint(2)=-footpoint(2); %flip the y data for our sanity...
    legpos(leg,:,t1)=footpoint(1:3);
return;


function bodypos=trackMovement()
global logfile info jpl legpos aibopos samplecount;

centroid=orbitCenter;
ydiff=mean(centroid(1:2,3))-mean(centroid(3:4,3));
xdiff=mean(centroid(1:2,1))-mean(centroid(3:4,1));
downrot=-atan(ydiff/xdiff)/pi*180;

buttons=zeros(4,samplecount);
for leg=1:4
  buttons(leg,:)=getButton(leg)';
end

bodypos=zeros(samplecount,6); % time, xyz pos | xyz rot
%pt=[0 0 0 1];
%M=eye(4);
%M=M*rotate(0,1,0,downrot); %standard 'cause our aibo points down usually and walks on flat ground
%for step=2:samplecount
%  delta=aibopos(:,:,step)-aibopos(:,:,step-1);
%  posdelta=legpos(:,:,step-1)-legpos(:,:,step);
%  posdelta=adjustDelta(M,posdelta);
%  buttonsOn=buttons(3,step) + buttons(4,step); % count of rear buttons on
% if(buttonsOn~=1) %if it only has 1 foot on the ground, it probably won't spin
%xdiff=delta([2 4],2)-delta([1 3],2);
%ydiff=delta([2 4],1)-delta([1 3],1);
% end
%%%%%%arrrr lost
%end

%%alternate sum method (straight forward distance traveled: GO!)
for step=2:samplecount
  delta=aibopos(:,:,step)-aibopos(:,:,step-1);
  dl=norm(delta(3,:));
  dr=norm(delta(4,:));
  rot=0;
  if(buttons(3,step))
    pairmag=dot(norm(delta(1,:)),norm(delta(3,:)));
    twist=[delta(1,1)-delta(3,1), delta(1,2)-delta(3,2), delta(1,3)-delta(3,3)];
    if(twist(1)==0)
      degtwist=0;
    else
      degtwist=atan(twist(2)/twist(1))/pi*180;
    end
    rot=rot+degtwist;
  end
  if(buttons(4,step))
    pairmag=dot(norm(delta(2,:)),norm(delta(4,:)));
    twist=[delta(2,1)-delta(4,1), delta(2,2)-delta(4,2), delta(2,3)-delta(4,3)];
    if(twist(1)==0)
      degtwist=0;
    else
      degtwist=atan(twist(2)/twist(1))/pi*180;
    end
    rot=rot-degtwist;
  end  
  bodypos(step,1)=dl+dr+bodypos(step-1,1);
  bodypos(step,4)=rot+bodypos(step-1,4);
end

step=1;
%while(step<samplecount){
steptimes=findStep(leg,step);
starting=steptimes(1); ending=steptimes(2);
plane=findStepPlane(3,starting,ending);
step=ending+1;
%end
  
  
function centers=orbitCenter()
global logfile info jpl legpos aibopos samplecount;
pos=zeros(3);
centers=permute(mean(permute(legpos,[3 1 2])),[2 3 1]) %yes it's a bitch, but it works
return;

function outpos=adjustDelta(M,inpos)
zeros(4,3);
for i=1:4
  pt=[inpos(i,:) 1];
  outpos(i,:)=pt*M;
end;

function drawBody(bodypos)
global logfile info jpl legpos aibopos samplecount pgraph;

figure(pgraph);

plot3(bodypos(:,1),bodypos(:,2),bodypos(:,3));

function steptimes=findStep(leg,from)
global logfile info jpl legpos aibopos samplecount pgraph;
buttons=getButton(leg);
hit=0; % 0 = waiting for hit, 1=while hit, 2=waiting for next hit
for i=from:samplecount
  switch(hit)
    case 0
      if(buttons(i))
	starting=i;
	hit=1;
      end
    case 1
      if(~buttons(i))
	hit=2;
      end
    case 2
      if(buttons(i))
	ending=i-1;
	hit=3;
      end
    otherwise
      break;
  end
end
steptimes=[starting,ending];

function stepplane=findStepPlane(leg,starting,ending)
global logfile info jpl legpos aibopos samplecount;
global points;
%we don't actually want z data
points=permute(legpos(leg,:,starting:ending),[3,2,1])
%least squares fitting:
%[m1 m2 b]*[x1 x2 x3 ~ ; y1 y2 y3 ~ ; 1 1 1 ~ ]=[z1 z2 z3 ~]
%A*B=C
%however A*B doesn't work 'cause different sizes, so multiply both sizes by A'??????????????
%B=[points(:,1:2),ones(size(points,1),1)]'
%C=points(:,3)'
%B=B*B'
%C=C*C'
%A=C\B
%consult mathworld! http://mathworld.wolfram.com/LeastSquaresFitting.html
%this should work, but there's an error somewhere else in the code.....????
x=points(:,1);
y=points(:,2);
b=cov(x,y)/var(x)^2;
a=mean(y)-b*mean(x);
stepplane=[a,b]
