% Create the E-shaped maze for the 2011 ARTSI robotics competition. Outputs
% a world source file (Emaze.ian) with drawing commands in the WorldBuilder
% format (coordinates are centers of objects) and comments giving the
% coordinates as endpoints in a form that can be turned into NEW_SHAPE
% commands in an fsm file.

function emaze
% parameter settings
outfile = 'Emaze.ian';
global f wallHeight tagHeight
wallHeight =  24 * 25.4;
westWall =    96 * 25.4;
northWall =   78 * 25.4;
alcoveWall =  30 * 25.4;
alcoveDepth = 46 * 25.4;
alcoveGap =    3 * 25.4;  % thickness of the walls separating adjacent alcoves
thresholdWidth = 20;      % width of pink threshold line
tagSize =      6 * 25.4;
tagSizeBig =   8 * 25.4;
tagHeight =    7 * 25.4;
tagAlcoveMargin = 3 * 25.4;  % distance from edge of wall to edge of tag
boxSize =      4 * 25.4;  % size of the boxes the robot is searching for
boxOrient =    0.2;       %orientation of target box

originX = -18 * 25.4;
originY =  18 * 25.4;
global curX curY heading
curX = originX; curY = originY;
heading = 0;

f = fopen(outfile,'w');
fprintf(f, '# environment settings\n');
fprintf(f, 'background material=CloudySky model=Dome\n');
fprintf(f, 'light location=[-100,3000,4000]');
fprintf(f, '\n');
fprintf(f, 'define Wall cube material=Wood\n');
fprintf(f, 'define Tag cube scale=[1,%f,%f]\n', tagSize, tagSize);
fprintf(f, '\n');

% draw the maze
fwd(westWall); turn(-90);
fwd(northWall); turn(-90);
fwd(alcoveWall); turn(-90); fwd(alcoveDepth); turn(90); fwd(alcoveGap); turn(90); fwd(alcoveDepth); turn(-90)
fwd(alcoveWall); turn(-90); fwd(alcoveDepth); turn(90); fwd(alcoveGap); turn(90); fwd(alcoveDepth); turn(-90)
fwd(alcoveWall); turn(-90)
fwd(northWall);
fprintf(f, '\n');

% draw the threshold lines
fprintf(f, 'define Threshold cube scale=[%f,%f,1] material=Pink collision=false\n', ...
    alcoveWall, thresholdWidth);
fprintf(f, 'Threshold location=[%f,%f,0.5]\n', ...
    originX+1*alcoveWall/2+0*alcoveGap, originY-northWall+alcoveDepth-thresholdWidth/2);
fprintf(f, 'Threshold location=[%f,%f,0.5]\n', ...
    originX+3*alcoveWall/2+1*alcoveGap, originY-northWall+alcoveDepth-thresholdWidth/2);
fprintf(f, 'Threshold location=[%f,%f,0.5]\n', ...
    originX+5*alcoveWall/2+2*alcoveGap, originY-northWall+alcoveDepth-thresholdWidth/2);
fprintf(f, '\n');

% tags at either end of the main corridor
makeTag( 0, originX+0*westWall, originY-(northWall-alcoveDepth)/2, tagSizeBig, 'n')
makeTag( 4, originX+1*westWall, originY-(northWall-alcoveDepth)/2, tagSizeBig, 's')

% main corridor tags opposite each alcove
makeTag( 1, originX+1*alcoveWall/2+0*alcoveGap, originY, tagSize, 'e')
makeTag( 2, originX+3*alcoveWall/2+1*alcoveGap, originY, tagSize, 'e')
makeTag( 3, originX+5*alcoveWall/2+2*alcoveGap, originY, tagSize, 'e')

% tags at the end of each alcove
makeTag(17, originX+1*alcoveWall/2+0*alcoveGap, originY-northWall, tagSize, 'w')
makeTag(12, originX+3*alcoveWall/2+1*alcoveGap, originY-northWall, tagSize, 'w')
makeTag( 7, originX+5*alcoveWall/2+2*alcoveGap, originY-northWall, tagSize, 'w')

outer = originY - (northWall-alcoveDepth) - tagAlcoveMargin - tagSize/2;
inner = originY - northWall + tagAlcoveMargin + tagSize/2;

% tags on the side walls of the alcove, just inside the entrance
makeTag(19, originX+0*alcoveWall+0*alcoveGap, outer, tagSize, 'n')
makeTag(15, originX+1*alcoveWall+0*alcoveGap, outer, tagSize, 's')
makeTag(14, originX+1*alcoveWall+1*alcoveGap, outer, tagSize, 'n')
makeTag(10, originX+2*alcoveWall+1*alcoveGap, outer, tagSize, 's')
makeTag( 9, originX+2*alcoveWall+2*alcoveGap, outer, tagSize, 'n')
makeTag( 5, originX+3*alcoveWall+2*alcoveGap, outer, tagSize, 's')


% tags on the side walls of the alcove, just before the dead end
makeTag(18, originX+0*alcoveWall+0*alcoveGap, inner, tagSize, 'n')
makeTag(16, originX+1*alcoveWall+0*alcoveGap, inner, tagSize, 's')
makeTag(13, originX+1*alcoveWall+1*alcoveGap, inner, tagSize, 'n')
makeTag(11, originX+2*alcoveWall+1*alcoveGap, inner, tagSize, 's')
makeTag( 8, originX+2*alcoveWall+2*alcoveGap, inner, tagSize, 'n')
makeTag( 6, originX+3*alcoveWall+2*alcoveGap, inner, tagSize, 's')

% now place the target boxes
fprintf(f, '\n');
fprintf(f, 'define Target cube scale=[%f,%f,%f] orientation=[0,0,%f]\n', ...
    boxSize, boxSize, boxSize, boxOrient);
targetY = originY - northWall + boxSize;
makeTarget(29, originX+1*alcoveWall+0*alcoveGap-boxSize, targetY, boxSize)
makeTarget(27, originX+2*alcoveWall+1*alcoveGap-boxSize, targetY, boxSize)
makeTarget(23, originX+2*alcoveWall+2*alcoveGap+boxSize, targetY, boxSize)
fclose(f);
end

function fwd(dist)
  global f curX curY heading wallHeight
  deltaX = dist * cos(heading);
  deltaY = dist * sin(heading);
  fprintf(f, '# From %7.1f , %7.1f  to  %7.1f , %7.1f\n', ...
        curX, curY, curX+deltaX, curY+deltaY);
  fprintf(f, 'Wall location=[%f,%f,%f] scale=[%f,1,%f] orientation=[0,0,%f]\n', ...
      curX+deltaX/2, curY+deltaY/2, wallHeight/2, dist, wallHeight, sin(heading/2));
  curX = curX + deltaX;
  curY = curY + deltaY;
end

function turn(degrees)
  global heading
  heading = heading + degrees/180*pi;
  if heading >= 2*pi
      heading = heading - 2*pi;
  else if heading < 0
          heading = heading + 2*pi;
      end
  end
end

function makeTag(id,x,y,size,orient)
  global f tagHeight
  switch orient
    case 'n'
      theta = 90;
      dx = 1; dy = 0;
    case 's'
      theta = 90;
      dx = -1; dy = 0;
    case 'e'
      theta = 0;
      dx = 0; dy = -1;
    case 'w'
      theta = 0;
      dx = 0; dy = 1;
    otherwise
      error('invalid orient argument to makeTag')
  end
  fprintf(f, '# Tag %2d at %7.1f , %7.1f\n', id, x+dx, y+dy);
  fprintf(f, 'Tag location=[%f,%f,%f] scale=[%f,1,%f] orientation=[0,0,%f] material="AprilTag/tag16h5/%02d"\n', ...
      x+dx, y+dy, tagHeight, size, size, cos(theta/2*pi/180), id);
end

function makeTarget(id,x,y,size)
  global f
  fprintf(f, '# Target %2d at %7.1f , %7.1f\n', id, x, y);
  fprintf(f, 'Target location=[%f,%f,%f] material="AprilTag/tag16h5/%02d"\n', ...
      x, y, size/2, id);
end

  
