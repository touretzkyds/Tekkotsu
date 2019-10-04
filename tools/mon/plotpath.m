global aibopos timetags window;
clf;
grid on;
hold on;
%T=viewmtx(-20,56);
T=viewmtx(0,0);
%wtf doesn't this work?
view(T);
specs=['m' 'g' 'r' 'b'];
timetags=zeros(4,2);
for leg=1:4
%%hack to plot leg position instead
bob=permute(aibopos(leg,:,:),[3 2 1]);%
%bob=permute(legpos(leg,:,:),[3 2 1]);
plot3(bob(:,1),bob(:,2),bob(:,3),specs(leg));
end
legend('lf','rf','lb','rb');
ylabel('left-right');
xlabel('back-forward');
zlabel('down-up');

for leg=1:4
bob=permute(aibopos(leg,:,:),[3 2 1]);
%bob=permute(legpos(leg,:,:),[3 2 1]);
timetags(leg,:)=plot3(bob(1,1),bob(1,2),bob(1,3), [specs(leg) 'x'], ...
    bob(end,1),bob(end,2),bob(end,3), [specs(leg) 'x'], 'MarkerSize',12)';
end
