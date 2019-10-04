function m = WalkCalibration(base_file_name,outname)
% WALKCALIBRATION Takes a set of actual and commanded speeds and
% performs linear least squares on a set of basis functions to
% determine proper calibration.
%
% Pass the base name of the input data set as first input and an
% output filename (if desired)
% Data files are broken into 6 categories:
% Forward-Strafe  (fs)
% Forward-Rotate  (fr)
% Strafe-Rotate   (sr)
% Backward-Rotate (br)
% Backward-Strafe (bs)
% Pure Rotation   (rr)
%
% So the file name for each should be BASEfs.txt, BASEfr.txt...
%
% The fs,fr,sr,rr categories are used to compute a forward matrix,
% and the sr,br,bs,rr categories are used to compute a backward
% matrix (since we assume left/right similarity, but
% forward/backward may not be a linear transition (and doesn't
% appear to be)
%
% First 3 columns of each file is the actual measured speed in
% forward, strafe, and rotation.  Columns 4-6 are the commanded
% values.

DEF_MAX_FORWARD=180;
DEF_MAX_BACKWARD=180;
DEF_MAX_STRAFE=140;
DEF_MAX_ROTATE=1.8;

dat{1}=load(strcat(base_file_name,'fs.txt'));
dat{2}=load(strcat(base_file_name,'fr.txt'));
dat{3}=load(strcat(base_file_name,'sr.txt'));
dat{4}=load(strcat(base_file_name,'br.txt'));
dat{5}=load(strcat(base_file_name,'bs.txt'));
dat{6}=load(strcat(base_file_name,'rr.txt'));

forward=[ dat{1} ; dat{2} ; dat{3} ; dat{6} ];
backward=[ dat{3} ; dat{4} ; dat{5} ; dat{6} ];

disp('Forward Matrix:');
intend=forward(:,1:3);
cmd=forward(:,4:6);
data=mungeRawData(intend,cmd);
m{1}=doLLS(data);

disp('10 Largest Errors:');
disp('First 3 cols are intended, next 3 are % error in command, last is total SSE');
calcmd=m{1}*data(1:3:size(data,1),1:(size(data,2)-1)/3)';
err=(cmd-calcmd');
scerr=err./[ones(size(err,1),1)*DEF_MAX_FORWARD ones(size(err,1),1)*DEF_MAX_STRAFE ones(size(err,1),1)*DEF_MAX_ROTATE/2];
err=[intend scerr (sum(scerr'.^2)')];
tmp=sortrows(err,7);
disp(tmp(end-10:end,:));
disp(sprintf('Total Error=%g;    N=%g;    Avg. Err=%g',sum(err(:,7)),size(intend,1),sum(err(:,7))/size(intend,1)));

disp(' ');

disp('Backward Matrix:');
intend=backward(:,1:3);
cmd=backward(:,4:6);
data=mungeRawData(intend,cmd);
m{2}=doLLS(data);

disp('10 Largest Errors:');
disp('First 3 cols are intended, next 3 are % error in command, last is total SSE');
calcmd=m{2}*data(1:3:size(data,1),1:(size(data,2)-1)/3)';
err=(cmd-calcmd');
scerr=err./[ones(size(err,1),1)*DEF_MAX_BACKWARD ones(size(err,1),1)*DEF_MAX_STRAFE ones(size(err,1),1)*DEF_MAX_ROTATE/2];
err=[intend scerr (sum(scerr'.^2)')];
tmp=sortrows(err,7);
disp(tmp(end-10:end,:));
disp(sprintf('Total Error=%g;    N=%g;    Avg. Err=%g',sum(err(:,7)),size(intend,1),sum(err(:,7))/size(intend,1)));

disp(' ');
disp('Final Forward Matrix:');
disp(m{1});
disp('Final Backward Matrix:');
disp(m{2});

%intend=forward(:,1:3);
%cmd=forward(:,4:6);
%disp([intend (m{1}*mungeRows(intend)')' ((m{1}*mungeRows(intend)')'-cmd)]);

%for visualization
% see basis function fits
%visFunc(m{1},mungeRows(dat{1}(:,1:3)),dat{1}(:,4:6),mungeRows(dat{2}(:,1:3)),dat{2}(:,4:6),mungeRows(dat{3}(:,1:3)),dat{3}(:,4:6),mungeRows(dat{6}(:,1:3)),dat{6}(:,4:6));
% see basis function errors
%visFuncError(m{1},mungeRows(dat{1}(:,1:3)),dat{1}(:,4:6),mungeRows(dat{2}(:,1:3)),dat{2}(:,4:6),mungeRows(dat{3}(:,1:3)),dat{3}(:,4:6),mungeRows(dat{6}(:,1:3)),dat{6}(:,4:6));
% see predictions
%visPredict(m{1},mungeRows(dat{1}(:,1:3)),dat{1}(:,4:6),mungeRows(dat{2}(:,1:3)),dat{2}(:,4:6),mungeRows(dat{3}(:,1:3)),dat{3}(:,4:6),mungeRows(dat{6}(:,1:3)),dat{6}(:,4:6));
% see errors
%visError(m{1},mungeRows(dat{1}(:,1:3)),dat{1}(:,4:6),mungeRows(dat{2}(:,1:3)),dat{2}(:,4:6),mungeRows(dat{3}(:,1:3)),dat{3}(:,4:6),mungeRows(dat{6}(:,1:3)),dat{6}(:,4:6));

disp('Max speeds are now:');
f=mungeRows([(0:DEF_MAX_FORWARD)' zeros(DEF_MAX_FORWARD+1,2)])*m{1}';
b=mungeRows([(-DEF_MAX_BACKWARD:0)' zeros(DEF_MAX_BACKWARD+1,2)])*m{2}';
fs=mungeRows([zeros(DEF_MAX_STRAFE*2+1,1) (-DEF_MAX_STRAFE:DEF_MAX_STRAFE)' zeros(DEF_MAX_STRAFE*2+1,1)])*m{1}';
bs=mungeRows([zeros(DEF_MAX_STRAFE*2+1,1) (-DEF_MAX_STRAFE:DEF_MAX_STRAFE)' zeros(DEF_MAX_STRAFE*2+1,1)])*m{2}';
fr=mungeRows([zeros(DEF_MAX_ROTATE*2/.01+1,2) (-DEF_MAX_ROTATE:.01:DEF_MAX_ROTATE)'])*m{1}';
br=mungeRows([zeros(DEF_MAX_ROTATE*2/.01+1,2) (-DEF_MAX_ROTATE:.01:DEF_MAX_ROTATE)'])*m{2}';

figure;
subplot(231);
plot(0:DEF_MAX_FORWARD,f(:,1));
title('Forward Velocity curve');xlabel('Actual');ylabel('Command')
subplot(232);
plot(-DEF_MAX_STRAFE:DEF_MAX_STRAFE,fs(:,2));
title('Forward Strafing Velocity curve');xlabel('Actual');ylabel('Command')
subplot(233);
plot(-DEF_MAX_ROTATE:.01:DEF_MAX_ROTATE,fr(:,3));
title('Forward Rotating Velocity curve');xlabel('Actual');ylabel('Command')
subplot(234);
plot(-DEF_MAX_BACKWARD:0,b(:,1));
title('Backward Velocity curve');xlabel('Actual');ylabel('Command')
subplot(235);
plot(-DEF_MAX_STRAFE:DEF_MAX_STRAFE,bs(:,2));
title('Backward Strafing Velocity curve');xlabel('Actual');ylabel('Command')
subplot(236);
plot(-DEF_MAX_ROTATE:.01:DEF_MAX_ROTATE,br(:,3));
title('Backward Rotating Velocity curve');xlabel('Actual');ylabel('Command')

%use bigger ranges in case calibration increases top speed value
fin=(0:DEF_MAX_FORWARD*2);
bin=(-DEF_MAX_BACKWARD*2:0);
stin=(-DEF_MAX_STRAFE*2:DEF_MAX_STRAFE*2);
rotin=(-DEF_MAX_ROTATE*2:.01:DEF_MAX_ROTATE*2);
f=mungeRows([fin' zeros(DEF_MAX_FORWARD*2+1,2)])*m{1}';
b=mungeRows([bin' zeros(DEF_MAX_BACKWARD*2+1,2)])*m{2}';
fs=mungeRows([zeros(DEF_MAX_STRAFE*4+1,1) stin' zeros(DEF_MAX_STRAFE*4+1,1)])*m{1}';
bs=mungeRows([zeros(DEF_MAX_STRAFE*4+1,1) stin' zeros(DEF_MAX_STRAFE*4+1,1)])*m{2}';
fr=mungeRows([zeros(DEF_MAX_ROTATE*4/.01+1,2) rotin'])*m{1}';
br=mungeRows([zeros(DEF_MAX_ROTATE*4/.01+1,2) rotin'])*m{2}';
bestf=find(f(:,1)<=DEF_MAX_FORWARD);
bestf=fin(bestf(end));
bestb=find(b(:,1)>=-DEF_MAX_BACKWARD);
bestb=bin(bestb(1));
bestfs1=find(fs(:,2)<=DEF_MAX_STRAFE);
bestfs2=find(fs(:,2)>=-DEF_MAX_STRAFE);
bestfs=min(abs([stin(bestfs1(end)) stin(bestfs2(1))]));
bestfr1=find(fr(:,3)<=DEF_MAX_ROTATE);
bestfr2=find(fr(:,3)>=-DEF_MAX_ROTATE);
bestfr=min(abs([rotin(bestfr1(end)) rotin(bestfr2(1))]));
bestbs1=find(bs(:,2)<=DEF_MAX_STRAFE);
bestbs2=find(bs(:,2)>=-DEF_MAX_STRAFE);
bestbs=min(abs([stin(bestbs1(end)) stin(bestbs2(1))]));
bestbr1=find(br(:,3)<=DEF_MAX_ROTATE);
bestbr2=find(br(:,3)>=-DEF_MAX_ROTATE);
bestbr=min(abs([rotin(bestbr1(end)) rotin(bestbr2(1))]));

disp(sprintf('         \t\tstrafe\trotate'));
disp(sprintf(' Forward:\t%.5g\t%.5g\t%.5g',bestf,bestfs,bestfr));
disp(sprintf('Backward:\t%.5g\t%.5g\t%.5g',bestb,bestbs,bestbr));
disp('These are only recommended values, based on default parameters');

if nargin>1
	disp(sprintf('Saving to %s',outname));
	maxspd=[ bestf abs(bestb) min([bestfs ; bestbs]) min([bestfr ; bestbr]) ];
	forwC=m{1};
	backC=m{2};
	save(outname,'maxspd','forwC','backC','-ASCII');
end

function row = mungeRows(itd)
abs_sr=abs(itd(:,2:3));

cross_mult=itd.*circshift(itd,[0 1]);

%ang=atan2(itd(:,2),itd(:,1));
ang=atan2(itd(:,2),abs(itd(:,1)));

spd=itd(:,1).^2+itd(:,2).^2;

invrot=itd(:,3).^(-1);
invrot(find(isinf(invrot)))=0;
invrot=invrot-itd(:,3);

gab=exp(-.5*itd(:,3).^2).*sin(itd(:,3)*2.5);

row=[itd abs_sr gab spd cross_mult(:,1:3) ones(size(itd,1),1) ];


function data = mungeRawData(itd,cmd)
row=mungeRows(itd);
data=zeros(3*size(row,1),size(row,2));
data(1:3:size(data,1),:)=row;
data=[ data circshift(data,1) circshift(data,2) reshape(cmd',size(data,1),1) ];



function ans = doLLS(data)
orig=data;

% nevermind about this - I guess it doesn't make a difference after
% all...
% % we scale by the measurement error for better accuracy
% % I figure my measurements are accurate to within 2.5 mm/s, or 1 degree/s
%scale=[ 2.5 2.5 1/180*pi 1 ];
%tmp=repmat(scale,size(data,1),3);
%tmp=[tmp repmat(scale(1:3)',size(data,1)/3,1)];
%data=data./tmp;
%disp([ getIntended(data./tmp)' getCommanded(data./tmp)' ]);

%now we pull out the big gun:
ans=data(:,1:size(data,2)-1)\data(:,end);
ans=reshape(ans,(size(data,2)-1)/3,3)';

% or you could do it the hard way:
%A=data(:,1:12);
%b=data(:,end);
%[U,S,V]=svd(A,0);
%c = U'*b;
%y = c./diag(S);
%x = V*y;
%e = A*x - b;
%disp(sprintf('Least Squares error is: %g',norm(e)));
%ans=reshape(x,4,3)';

% nevermind...
% % and don't forget to unscale
%tmp=[ scale/scale(1) ; scale/scale(2) ; scale/scale(3) ];
%ans=ans./tmp;

%for visualization
t1=getIntended(orig)';
t2=getCommanded(orig)';
% see predictions
%visPredict(t1,t2,ans);
% see errors
%visError(t1,t2,ans);

function ans = getIntended(data)
ans=data(1:3:size(data,1),1:((size(data,2)-1)/3))';

function ans = getCommanded(data)
ans=reshape(data(:,end),3,size(data,1)/3);


function visFunc(m,varargin)
srcs=[1 2 3 2 3 3 1 1 2 3 0];
visFuncs(m,[1 2 3],srcs,varargin{:});
last=4;
if(size(m,2)>=7)
	visFuncs(m,4:7,srcs,varargin{:})
	last=8;
end
visFuncs(m,last:size(m,2),srcs,varargin{:})

function visFuncError(m,varargin)
srcs=[1 2 3 2 3 3 1 1 2 3 0];
visFuncErrors(m,[1 2 3],srcs,varargin{:});
last=4;
if(size(m,2)>=7)
	visFuncErrors(m,4:7,srcs,varargin{:})
	last=8;
end
visFuncErrors(m,last:size(m,2),srcs,varargin{:})

function visPredict(m,varargin)
visPredicts(m,[1 2 3],varargin{:});
last=4;
if(size(m,2)>=7)
	visPredicts(m,4:7,varargin{:})
	last=8;
end
visPredicts(m,last:size(m,2),varargin{:})

function visError(m,varargin)
visErrors(m,[1 2 3],varargin{:});
last=4;
if(size(m,2)>=7)
	visErrors(m,4:7,varargin{:})
	last=8;
end
visErrors(m,last:size(m,2),varargin{:})

function visFuncs(m,jrange,xrange,varargin)
figure;
colors={ 'b.' 'r.' 'g.' 'm.' };
for j=jrange
	intend=[];
	for n=1:2:size(varargin,2),
		intend=[intend ; varargin{n}];
	end
	if(xrange(j)==0)
		mn=0;
		mx=0;
	else
		mn=min(intend(:,xrange(j)));
		mx=max(intend(:,xrange(j)));
	end
	for i=1:3,
		subplot(3,size(jrange,2),(j-min(jrange)+1)+(i-1)*size(jrange,2));
		if(mn==mx)
			x=[mn-1 mx+1];
			y=[ m(i,j) m(i,j) ];
		else
			x=mn:(mx-mn)/100:mx;
			dat=circshift([ x ; zeros(size(x)) ; zeros(size(x)) ],xrange(j)-1)';
			mdat=mungeRows(dat);
			y=mdat(:,j)*m(i,j);
		end
		plot(x,y,'k-');
		hold on;
		for n=1:2:size(varargin,2),
			t1=varargin{n};
			t2=varargin{n+1};
			if(xrange(j)==0)
				plot(zeros(size(t1,1)),t2(:,i)-(m(i,:)*t1')'+t1(:,j)*m(i,j),colors{(n+1)/2});
			else
				plot(t1(:,xrange(j)),t2(:,i)-(m(i,:)*t1')'+t1(:,j)*m(i,j),colors{(n+1)/2});
			end
		end
	end
end

function visFuncErrors(m,jrange,xrange,varargin)
figure;
colors={ 'b.' 'r.' 'g.' 'm.' };
for j=jrange
	intend=[];
	for n=1:2:size(varargin,2),
		intend=[intend ; varargin{n}];
	end
	if(xrange(j)==0)
		mn=0;
		mx=0;
	else
		mn=min(intend(:,xrange(j)));
		mx=max(intend(:,xrange(j)));
	end
	if(mn==mx)
		x=[mn-1 mx+1];
	else
		x=[mn mx];
	end
	for i=1:3,
		subplot(3,size(jrange,2),(j-min(jrange)+1)+(i-1)*size(jrange,2));
		plot(x,[0 0],'k-');
		hold on;
		for n=1:2:size(varargin,2),
			t1=varargin{n};
			t2=varargin{n+1};
			if(xrange(j)==0)
				plot(zeros(size(t1,1)),t2(:,i)-(m(i,:)*t1')',colors{(n+1)/2});
			else
				plot(t1(:,xrange(j)),t2(:,i)-(m(i,:)*t1')',colors{(n+1)/2});
			end
		end
	end
end

function visPredicts(m,jrange,varargin)
figure;
colors={ 'b.' 'r.' 'g.' 'm.' };
for j=jrange
	intend=[];
	for n=1:2:size(varargin,2),
		intend=[intend ; varargin{n}];
	end
	mn=min(intend(:,j));
	mx=max(intend(:,j));
	for i=1:3,
		subplot(3,size(jrange,2),(j-min(jrange)+1)+(i-1)*size(jrange,2));
		if(mn==mx)
			x=[mn-1 mx+1];
			y=[ m(i,j) m(i,j) ];
		else
			x=[mn mx];
			y=x*m(i,j);
		end
		plot(x,y,'k-');
		hold on;
		for n=1:2:size(varargin,2),
			t1=varargin{n};
			t2=varargin{n+1};
			plot(t1(:,j),t2(:,i)-(m(i,:)*t1')'+t1(:,j)*m(i,j),colors{(n+1)/2});
		end
	end
end

function visErrors(m,jrange,varargin)
figure;
colors={ 'b.' 'r.' 'g.' 'm.' };
for j=jrange
	intend=[];
	for n=1:2:size(varargin,2),
		intend=[intend ; varargin{n}];
	end
	mn=min(intend(:,j));
	mx=max(intend(:,j));
	if(mn==mx)
		x=[mn-1 mx+1];
	else
		x=[mn mx];
	end
	for i=1:3,
		subplot(3,size(jrange,2),(j-min(jrange)+1)+(i-1)*size(jrange,2));
		plot(x,[0 0],'k-');
		hold on;
		for n=1:2:size(varargin,2),
			t1=varargin{n};
			t2=varargin{n+1};
			plot(t1(:,j),t2(:,i)-(m(i,:)*t1')',colors{(n+1)/2});
		end
	end
end
