global timeslider pgraph jgraph trtimetags legpos;
     mint=logfile(1,1);
     maxt=logfile(end,1);
     p=get(timeslider,'value');
     view=window.size/window.zoomfactor;
window.start=p*(maxt-mint-view)+mint;
view=window.size/window.zoomfactor;
window.stop=window.start+view;
set(timeslider,'sliderstep',[view/5/window.size,view/window.size]);
for i=1:length(subp)
     ax=axis(subp(i));
     axis(subp(i),[window.start,window.stop, ax(3),ax(4)]);
end

global aibopos timetags updatego;
%if(updatego>0)
posstart=min(find(logfile(:,1) >= window.start));
posend=max(find(logfile(:,1) <= window.stop));
startframe=aibopos(:,:,posstart);
stopframe=aibopos(:,:,posend);
trstartframe=legpos(:,:,posstart);
trstopframe=legpos(:,:,posend);
if(ishandle(timetags(1,1))>0)
for leg=1:4
set(timetags(leg,1),'XData',startframe(leg,1));
set(timetags(leg,1),'YData',startframe(leg,2));
set(timetags(leg,1),'ZData',startframe(leg,3));
set(timetags(leg,2),'XData',stopframe(leg,1));
set(timetags(leg,2),'YData',stopframe(leg,2));
set(timetags(leg,2),'ZData',stopframe(leg,3));

%%trajectory

set(trtimetags(leg,1),'XData',trstartframe(leg,1));
set(trtimetags(leg,1),'YData',trstartframe(leg,2));
set(trtimetags(leg,1),'ZData',trstartframe(leg,3));
set(trtimetags(leg,2),'XData',trstopframe(leg,1));
set(trtimetags(leg,2),'YData',trstopframe(leg,2));
set(trtimetags(leg,2),'ZData',trstopframe(leg,3));

end
end
%end
