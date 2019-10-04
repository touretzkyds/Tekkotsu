info.FrameTime=8;
info.NumFrames=4;
info.SlowFrameTime=128;
info.NumSlowFrames=1;
info.SoundBufferTime=32;
info.NumPIDJoints=18;
info.NumBinJoints=2;
info.NumLEDs=22;
info.NumOutputs=info.NumPIDJoints + info.NumBinJoints + info.NumLEDs;
info.isFastOutput=zeros(info.NumOutputs)+1; %this is a lie. but who cares?
info.isRealERS220=info.isFastOutput; %also a lie
info.JointsPerLeg=3;
info.NumLegs=4;
info.NumLegJoints=info.JointsPerLeg*info.NumLegs;
info.NumHeadJoints=3;
info.NumTailJoints=2;
info.NumMouthJoints=1;
info.NumEarJoints=2;
info.NumButtons=11;
info.NumSensors=1+3+1+5;  % 1 dist, 3 accel, 1 thermo, 5 from power, see SensorOffset_t
% offset data
info.legOrder={'lf'  'rf'  'lb'  'rb'};
info.legJointOrder={'Rotator' 'Elevator' 'Knee'};
info.headJointOrder={'Tilt' 'Pan' 'Roll'};
info.LogOffset.positions=2;
info.LogOffset.duties=info.LogOffset.positions+info.NumPIDJoints;
info.LogOffset.sensors=info.LogOffset.duties+info.NumPIDJoints;
info.LogOffset.buttons=info.LogOffset.sensors+6;
info.SensorOffsetOf.IRDist=0;
info.SensorOffsetOf.BAccelOffset=1;
info.SensorOffsetOf.LAccelOffset=2;
info.SensorOffsetOf.DAccelOffset=3;

%%%%old
%info.limbs.f_body_to_shoulder=[59.5 59.2 0];
%info.limbs.f_leg_shoulder_to_knee=[12.8 .5 -64];
%info.limbs.f_leg_knee_to_ball=[-18 0 -67.23];
%info.limbs.h_body_to_shoulder=[59.5 59.2 0];
%info.limbs.h_leg_shoulder_to_knee=[-12.8 .5 -64];
%info.limbs.h_leg_knee_to_ball=[-18 0 -74.87];


%note: these coordinates are off 90deg (forward left up)
info.limbs.f_body_to_shoulder=[59.5 59.2 0];
info.limbs.f_leg_shoulder_to_knee=[12.8 .5 -64];
info.limbs.f_leg_knee_to_ball=[-18 0 -67.23];
info.limbs.h_body_to_shoulder=[-59.5 59.2 0];
info.limbs.h_leg_shoulder_to_knee=[-12.8 .5 -64];
info.limbs.h_leg_knee_to_ball=[-18 0 -74.87];

%format: leg,joint,xyz (where +x=forward, +y=left, +z=up)
info.limblen=zeros(4,3,3);
info.limblen(1,:,:)=[ info.limbs.f_body_to_shoulder; info.limbs.f_leg_shoulder_to_knee; info.limbs.f_leg_knee_to_ball ] ; %lf 
%%%right is mirror of left
info.limblen(2,:,:)=[info.limblen(1,:,1)' , -info.limblen(1,:,2)' , info.limblen(1,:,3)' ]; %rf
info.limblen(3,:,:)=[info.limbs.h_body_to_shoulder; info.limbs.h_leg_shoulder_to_knee; info.limbs.h_leg_knee_to_ball ]  ; %lb
info.limblen(4,:,:)=[info.limblen(3,:,1)' , -info.limblen(3,:,2)' , info.limblen(3,:,3)' ]; %rf

info.walkheight=160;
