function plotpos
global aibopos trtimetags window plotops legpos bodypos;
clf;
grid on;
hold on;
T=viewmtx(0,0);
%wtf doesn't this work?
view(T);
specs=['m' 'g' 'r' 'b'];
timetags=zeros(4,2);
for leg=1:4
  %%draw leg trajectory
  bob=permute(legpos(leg,:,:),[3 2 1]);
  plot3(bob(:,1),bob(:,2),bob(:,3),specs(leg));
end
if(plotops.drawbuttons~=0)
  for leg=1:4
    %%plot button-on
    bob=permute(legpos(leg,:,:),[3 2 1]);
    button=getButton(leg);
    ontime=extractButtonOn(button);
    hold on;
    plot3(bob(ontime,1),bob(ontime,2),bob(ontime,3),[specs(leg) 'o']);
  end
end
legend('lf','rf','lb','rb');
ylabel('left-right');
xlabel('back-forward');
zlabel('down-up');

%add time ticks
for leg=1:4
bob=permute(legpos(leg,:,:),[3 2 1]);
trtimetags(leg,:)=plot3(bob(1,1),bob(1,2),bob(1,3), [specs(leg) 'x'], ...
    bob(end,1),bob(end,2),bob(end,3), [specs(leg) 'x'], 'MarkerSize',32)';
end

%plot heading

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
