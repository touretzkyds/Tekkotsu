#if defined(TGT_HAS_CAMERA) || defined(TGT_HAS_WEBCAM)

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Shared/ProjectInterface.h"
#include "Vision/RegionGenerator.h"
#include "Vision/SegmentedColorGenerator.h"

//! Outputs some basic region statistics regarding the current frame to the console (and then stops again)
class RegionStatsBehavior : public BehaviorBase {
public:
	//! constructor
	RegionStatsBehavior() : BehaviorBase("RegionTest") {}
	
	virtual void doStart() {
		//long line: get the regions from the default region generator
		//have to cast it to the type actually returned by the generator in question (see documentation)
		RegionGenerator::region_stats * regions=(RegionGenerator::region_stats*)ProjectInterface::defRegionGenerator->getImage(ProjectInterface::fullLayer,0);
		unsigned int num_colors=ProjectInterface::defSegmentedColorGenerator->getNumColors();
		
		//do some processing on the regions
		for(unsigned int i=0; i<num_colors; i++) {
		
			std::cout << "There are " << regions[i].num << " " << regions[i].name << " regions:" << std::endl;
			
			if(regions[i].num>0 && regions[i].list!=NULL) { //really only need to check one
				//region list is sorted by size, so first entry (if list isn't empty) is the largest:
				std::cout << "\tlargest region has area " << regions[i].list->area << std::endl;
				//but you might prefer the total size of all regions of that color:
				std::cout << "\ttotal area is " << regions[i].total_area << std::endl;
			}
		}
		
		//just do it once for the current frame at launch, now stop again
		stop();
	}

	static std::string getClassDescription() { return "Outputs some basic region statistics regarding the current frame to the console (and then stops again)"; }
	virtual std::string getDescription() const { return getClassDescription(); }
	
};

REGISTER_BEHAVIOR_MENU_OPT(RegionStatsBehavior,DEFAULT_TK_MENU"/Vision Demos",BEH_NONEXCLUSIVE);

#endif

/*! @file
 * @brief Defines RegionStatsBehavior, which outputs some basic region statistics to the console
 * @author ejt (Creator)
 */
