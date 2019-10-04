#include "BallDetectionGenerator.h"
#include "Events/EventRouter.h"
#include "Shared/Util.h"
#include "Wireless/Wireless.h"
#include "Shared/Config.h"
#include "Events/VisionObjectEvent.h"
#include "Shared/Profiler.h"

#include "Vision/SegmentedColorGenerator.h"
#include "Vision/RegionGenerator.h"
typedef RegionGenerator::region region; //!< shorthand - using CMVision regions
typedef SegmentedColorGenerator::color_class_state color_class_state; //!< shorthand - using CMVision color structs

BallDetectionGenerator::BallDetectionGenerator(unsigned int mysid, const RegionGenerator * rg, unsigned int colorIdx, unsigned int threshmapChan, unsigned int noiseFiltering, float confidence)
	: EventGeneratorBase("BallDetectionGenerator",EventBase::visObjEGID,mysid,rg->getGeneratorID(),rg->getSourceID(),EventBase::statusETID), clrIdx(colorIdx), tmIdx(threshmapChan), ball(), present(false), count(0), noiseThreshold(noiseFiltering), confidenceThreshold(confidence)
{}

void
BallDetectionGenerator::doEvent() {
	PROFSECTION("BallDetection::doEvent()",*mainProfiler);
	EventGeneratorBase::doEvent();
	if(event->getGeneratorID()!=getListenGeneratorID() || event->getSourceID()!=getListenSourceID())
		return;

	const SegmentedColorFilterBankEvent * segev=dynamic_cast<const SegmentedColorFilterBankEvent*>(event);
	if(NULL==segev) {
		serr->printf("BallDetectionGenerator's event %s was not a SegmentedColorFilterBankEvent",event->getName().c_str());
		return;
	}
	
  static const bool debug_ball = false;
  static const bool debug_conf = false;

  static int frame_cnt=0;
  static const int print_period=1;

  if(debug_ball)
    frame_cnt = (frame_cnt + 1) % print_period;

	unsigned int layer=segev->getNumLayers()-config->vision.resolution-1;
	if(segev->getNumColors()<=clrIdx) {
		serr->printf("BallDetectionGenerator::clrIdx %d is invalid (only %d available)\n",clrIdx,segev->getNumColors());
		return;
	}
	if(segev->getNumChannels()<=tmIdx) {
		serr->printf("BallDetectionGenerator::tmIdx %d is invalid (only %d available)\n",tmIdx,segev->getNumChannels());
		return;
	}
	if(segev->getNumLayers()<=layer) {
		serr->printf("BallDetectionGenerator's selected layer %d is invalid (only %d available)\n",layer,segev->getNumLayers());
		return;
	}
	if(segev->getImage(layer,tmIdx)==NULL) {
		serr->printf("BallDetectionGenerator source's getImage returned NULL!\n");
		return;
	}
  const color_class_state& ballCCS=reinterpret_cast<const color_class_state*>(segev->getImage(layer,tmIdx))[clrIdx];

  ball.confidence = 0;
	region * ball_region=NULL;

  region * or_reg=ballCCS.list; //head of linked list of regions of target color. or_reg= original region (without noise removal)
  if(!or_reg){
		count++;
		// The next 3 lines could be cleared if no dimensions are sent with the deactivation events
		float dim=segev->getWidth(layer);
		//the max width and height of scaled dimensions
		float cw=segev->getWidth(layer)/dim;
		float ch=segev->getHeight(layer)/dim;	
		unsigned int frame_number=segev->getFrameNumber();
		if (present && count>noiseThreshold) {  // If there are no regions a number of frames and ball was present
			count=0;
			present=false; // Ball is not present
			createEvent(EventBase::deactivateETID,0,0,0,0,0,cw,ch,frame_number); // Deactivation Events
		}
		return;
  }
  
  unsigned int n = 0;
  while(or_reg && n<NUM_CHECK) {
    //float conf,conf0,conf_square_bbox,conf_area,conf_green,conf_area_bonus;
    //float conf_red_v_area;
    //int edge;

    int w = or_reg->x2 - or_reg->x1 + 1;
    int h = or_reg->y2 - or_reg->y1 + 1;
    
    int edge = calcEdgeMask(or_reg->x1,or_reg->x2,or_reg->y1,or_reg->y2,segev->getWidth(layer),segev->getHeight(layer));
    float conf0 = (w >= 3) * (h >= 3) * (or_reg->area >= 7);
    float conf_square_bbox = 
      edge ?
      gaussian_with_min(pct_from_mean(w,h) / .6f, 1e-3f) :
      gaussian_with_min(pct_from_mean(w,h) / .2f, 1e-3f);
    float conf_area =
      edge ?
      gaussian_with_min(pct_from_mean(((float)M_PI)*w*h/4.0f,or_reg->area) / .6f, 1e-3f) :
      gaussian_with_min(pct_from_mean(((float)M_PI)*w*h/4.0f,or_reg->area) / .2f, 1e-3f);
    float conf_area_bonus = or_reg->area / 1000.0f;

    float conf = conf0*conf_square_bbox*conf_area + conf_area_bonus;

    if(conf > 1.0) conf = 1.0f;
    
    if(debug_conf && frame_cnt == 0) {
      printf("conf0 %g conf_square_bbox %g conf_area %g conf_area_bonus %g final %g\n",
						 conf0,conf_square_bbox,conf_area,conf_area_bonus,conf);
    }
    
		/*
			float green_f;
			if(conf > ball->confidence) {
      if(debug_ball &&
			(frame_cnt == 0 ||
			frame_cnt == 1)) {
			printf("%s ball n%d cen (%f,%f) area %d bbox (%d,%d)-(%d,%d) conf %f\n", (ball_color==COLOR_PINK)?"Pink":"Orange",
			n,or_reg->cen_x,or_reg->cen_y,or_reg->area,
			or_reg->x1,or_reg->y1,or_reg->x2,or_reg->y2,
			conf);
      }

      int sx[2],sy[2]; // scan bounding coordinates;
      bool edge_x[2],edge_y[2]; // true if scan coordinate went off edge
      static const int scan_distance = 3;
      static int color_cnts[num_colors];
      int scan_pixels;
      int good_value;
    
      sx[0] = or_reg->x1 - scan_distance;
      edge_x[0] = (sx[0] < 0);
      if(edge_x[0]) sx[0] = 0;
      sx[1] = or_reg->x2 + scan_distance;
      edge_x[1] = (sx[1] > width-1);
      if(edge_x[1]) sx[1] = width-1;
      sy[0] = or_reg->y1 - scan_distance;
      edge_y[0] = (sy[0] < 0);
      if(edge_y[0]) sy[0] = 0;
      sy[1] = or_reg->y2 + scan_distance;
      edge_y[1] = (sy[1] > height-1);
      if(edge_y[1]) sy[1] = height-1;

      scan_pixels = 0;
      for(int color_idx=0; color_idx<MAX_COLORS; color_idx++)
			color_cnts[color_idx]=0;
    
      // do horizontal strips
      for(int side=0; side<2; side++) {
			if(!edge_y[side]) {
			scan_pixels+=addToHistHorizStrip(sy[side],sx[0],sx[1],color_cnts);
			}
      }
    
      // do vertical strips
      for(int side=0; side<2; side++) {
			if(!edge_x[side]) {
			scan_pixels+=addToHistVertStrip(sx[side],sy[0],sy[1],color_cnts);
			}
      }

      int non_robot_fringe=0;
      non_robot_fringe += 5*color_cnts[getColor("blue")];
      non_robot_fringe -=   color_cnts[getColor("green")];

      conf_red_v_area = 1 + non_robot_fringe / or_reg->area;
      conf_red_v_area = bound(conf_red_v_area,0.0,1.0);
      
      good_value = 0;
      good_value += 2*color_cnts[getColor("blue")];
      good_value += 3*color_cnts[getColor("green")]/2;

      green_f = std::max(good_value+1,1) / (scan_pixels + 1.0);
      green_f = bound(green_f,0.0,1.0);
      conf_green = green_f;
    
      conf = 
			conf0 *
			conf_square_bbox *
			conf_area *
			+ conf_area_bonus;
    
      if(conf > 1.0) conf = 1.0;
			}*/
    
    if(conf > ball.confidence) {
      /*
				d = sqrt((FocalDist * FocalDist * YPixelSize) * (M_PI * BallRadius * BallRadius) /
				(or_reg->area));
    
				vector3d ball_dir; // direction of ball from camera in robot coordinates
				ball_dir = getPixelDirection(or_reg->cen_x,or_reg->cen_y);
    
				// Reject if ball above level plane
				//      if(atan2(ball_dir.z,hypot(ball_dir.x,ball_dir.y)) <= (5*(M_PI/180))) {
        ball->edge = calcEdgeMask(or_reg);
        
        vector3d intersect_ball_loc,pixel_size_ball_loc;

        bool intersects=false;
        vector3d intersection_pt(0.0,0.0,0.0);
        intersects=GVector::intersect_ray_plane(camera_loc,ball_dir,
				vector3d(0.0,0.0,BallRadius),vector3d(0.0,0.0,1.0),
				intersection_pt);
        if(intersects) {
				intersect_ball_loc = intersection_pt;
        }

        pixel_size_ball_loc = camera_loc + ball_dir * d;

        vector3d ball_loc,alt_ball_loc;
        if(ball->edge!=0 && intersects) {
				ball_loc     = intersect_ball_loc;
				alt_ball_loc = pixel_size_ball_loc;
        } else {
				alt_ball_loc = intersect_ball_loc;
				ball_loc     = pixel_size_ball_loc;
        }
			*/ 

			ball.confidence = conf;
        
			//        ball->loc = ball_loc;
        
			//        ball->distance = hypot(ball_loc.x,ball_loc.y);
        
			//        findSpan(ball->left,ball->right,or_reg->x1,or_reg->x2,or_reg->y1,or_reg->y2);

			ball_region = or_reg;

			/*        if(debug_ball &&
								frame_cnt==0) {
								printf("###found ball, conf %g loc (%g,%g,%g) alt (%g,%g,%g) dist %g left %g right %g edge %d\n",
								ball->confidence,
								ball->loc.x,ball->loc.y,ball->loc.z,
								alt_ball_loc.x,alt_ball_loc.y,alt_ball_loc.z,
								ball->distance,ball->left,ball->right,ball->edge);
								}*/
			// }
    }

    or_reg = or_reg->next;
    n++;
  }
  //return (ball_color==getColor("red"))?generateEvent (VisionObjectEvent::RedBallSID, ball->confidence, ball_region->cen_x, ball_region->cen_y):generateEvent(VisionObjectEvent::PinkBallSID,ball->confidence,ball_region->cen_x,ball_region->cen_y);
  
	testSendEvent(*segev,(float)ball.confidence,(int)ball_region->x1,(int)ball_region->x2,(int)ball_region->y1,(int)ball_region->y2,(int)ball_region->area); 
}

void
BallDetectionGenerator::testSendEvent(const FilterBankEvent& ev, float conf, int regX1,int regX2,int regY1,int regY2, int area) {
	unsigned int layer=ev.getNumLayers()-config->vision.resolution-1;

	// x will be scaled to [-1,1], y scaled correspondingly to [ -1/aspectRatio , 1/aspectRatio ]
	float dim=ev.getWidth(layer);

	//the max width and height of scaled dimensions
	float cw=config->vision.x_range;
	float ch=config->vision.y_range;

	//scale the dimensions
	float cx1=2.0f*regX1/dim - cw;
	float cx2=2.0f*(regX2+1)/dim - cw;
	float cy1=2.0f*regY1/dim - ch;
	float cy2=2.0f*(regY2+1)/dim - ch;
	float carea=4.0f*area/(dim*dim);
	unsigned int frame_number=ev.getFrameNumber();	

	if (conf>confidenceThreshold) {
		if (present) {
			count=0;
			createEvent(EventBase::statusETID,cx1,cx2,cy1,cy2,carea,cw,ch,frame_number);
		} else {
			count++;
			if (count>noiseThreshold) {
				count=0;
				present=true;
				createEvent(EventBase::activateETID,cx1,cx2,cy1,cy2,carea,cw,ch,frame_number);
			}
		}
	} else {
		if (!present) {
			count=0;
		} else {
			count++;
			if (count>noiseThreshold) {
				count=0;
				present=false;
				createEvent(EventBase::deactivateETID,0,0,0,0,carea,cw,ch,frame_number);
			} else {
				createEvent(EventBase::statusETID,cx1,cx2,cy1,cy2,carea,cw,ch,frame_number);
			}
		}
	}
}

void
BallDetectionGenerator::createEvent(EventBase::EventTypeID_t etid, float bbX1,float bbX2,float bbY1,float bbY2,float area,float rx,float ry,unsigned int frame) const {
	VisionObjectEvent vo(mySourceID,etid,bbX1,bbX2,bbY1,bbY2,area,rx,ry,frame);
	vo.setName(getName());
	erouter->postEvent(vo);
}

int
BallDetectionGenerator::calcEdgeMask(int x1,int x2,int y1,int y2, int width, int height) {
  static const int boundary_pixel_size=1;

  int edge = 0;
  if(x1 <= 0       +boundary_pixel_size) edge |= OFF_EDGE_LEFT  ;
  if(x2 >= width -1-boundary_pixel_size) edge |= OFF_EDGE_RIGHT ;
  if(y1 <= 0       +boundary_pixel_size) edge |= OFF_EDGE_TOP   ;
  if(y2 >= height-1-boundary_pixel_size) edge |= OFF_EDGE_BOTTOM;

  return edge;
}

/*! @file 
 * @brief Implements BallDetectionGenerator, which uses segmented color region information to detect round objects
 * @author alokl (Creator)
 * @author ejt (reorganized)
 * @author Ignacio Herrero Reder < nhr at dte uma es > (VisionObjectInfo Boundary Box - bug 74)
 *
 * History is old, may have grown from CMPack (CMU Robosoccer) roots?
 * I think if there's any of their code left, it's probably *mostly*
 * the commented out stuff I (ejt) left for posterity when
 * reorganizing.  But alokl didn't flag this as CMPack's prior to
 * inital release, and they didn't request credit for it when they
 * reviewed the code, so I guess it's all ours...
 */
