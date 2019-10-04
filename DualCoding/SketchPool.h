//-*-c++-*-
#ifndef INCLUDED_SketchPool_h
#define INCLUDED_SketchPool_h

#include <vector>
#include <iostream>
#include <sstream> // for ostringstream

#include "SketchPoolRoot.h"

namespace DualCoding {

class SketchSpace;
class SketchDataRoot;
template<class T> class SketchData;

//! Manages a pool of SketchData<T> instances

template<typename T>
class SketchPool : public SketchPoolRoot {
public:
  //! this is made public so VisualRoutinesBehavior can access
  std::vector<SketchData<T>*> elements;
  
  //! Constructor
  SketchPool<T>(SketchSpace *_space, const std::string& _name, int poolsize = 0); 
  
	//! Destructor
  ~SketchPool<T>();
  
	//! Delete all sketches in the pool; commplain if refcount nonzero.  Used by destructor and by SketchSpace::resize()
	void deleteElements();

  //!  Make all sketches non-viewable, hence reclaimable when refcount drops to zero
  void clear(bool clearRetained);

  SketchData<T>* getFreeElement(void); 
  
  SketchData<T>* findSketchData(const std::string &name);

  //! Returns a list of the valid SketchData's in this pool.
  std::string getSketchListForGUI();
  
  //! Returns a copy of the sketch with specified ID, null if no such Sketch.
  SketchDataRoot* retrieveSketch(int id);
  
  void dumpPool() const;

 private:
  // typename for iteration over elements
  typedef typename std::vector<SketchData<T>*>::const_iterator CI;

  SketchPool(const SketchPool&); //<! never call this
  SketchPool& operator=(const SketchPool&); //!< never call this
};

// **************** Implementation ****************

template <class T>
SketchPool<T>::SketchPool(SketchSpace *_space, const std::string& _name, int poolsize) :
  SketchPoolRoot(_space,_name),
  elements(std::vector<SketchData<T>*>(poolsize)) 
{
  for (int i=0; i<poolsize; i++) {
    elements[i] = new SketchData<T>(space);
  };
}

template <class T>
SketchPool<T>::~SketchPool() {
	deleteElements();
}

template <class T>
void SketchPool<T>::deleteElements() {
  for (unsigned int i = 0; i < elements.size(); i++)
    if(elements[i]->refcount > 0)
      printf("ERROR in ~SketchPool<T>: Element %d [%p] has ref_count == %d != 0\n",
						 i,elements[i],elements[i]->refcount);
    else
      delete elements[i];
	elements.clear();
}

template <class T>
void SketchPool<T>::clear(bool clearRetained) {
  for (CI it = elements.begin(); it != elements.end(); it++ ) {
    /*
    cout << "clear " << (*it)->space->name << "  " << (*it)->id
	 << " " << (*it)->name << " refcount=" << (*it)->refcount
	 << " refreshTag=" << (*it)->refreshTag
	 << " refCntr=" << getRefreshCounter()
	 << " viewable=" << (*it)->viewable
	 << " clrpend=" << (*it)->clearPending << endl;
    */
    if ( clearRetained )
      (*it)->retain(false);
    if ( !(*it)->retained )
      (*it)->setViewable(false);
    if ( (*it)->refreshTag < getRefreshCounter() )
      (*it)->clearPending = false;
    else
      (*it)->clearPending = true;
  }
}

template <class T>
SketchData<T>* SketchPool<T>::getFreeElement(void) 
{
  for (CI it = elements.begin(); it != elements.end(); it++ ) {
    if ( (*it)->refcount == 0 && !(*it)->retained &&
	 (*it)->viewable == false && (*it)->clearPending == false )
      return *it;
    else if ( (*it)->refcount < 0 )
      std::cerr << "PROBLEM: negative refcount" << std::endl;
  };
  SketchData<T>* res = new SketchData<T>(space);
  elements.push_back(res);
  return res;
}

template<class T>
SketchData<T>* SketchPool<T>::findSketchData(const std::string &sketchname) {
  for (CI it = elements.begin(); it != elements.end(); it++ )
    if ( ((*it)->refcount > 0 || (*it)->retained || (*it)->viewable) && (*it)->name == sketchname )
		return *it;
  return NULL;
}
	 

template<class T>
std::string SketchPool<T>::getSketchListForGUI()
{
	std::ostringstream liststream;
	for ( unsigned int i = 0; i < elements.size(); i++ ) {
		if ( elements[i]->clearPending ) {
			elements[i]->setViewable(false);
			elements[i]->clearPending = false;
		}
		else if ( elements[i]->isViewable() ) {
			elements[i]->refreshTag = getRefreshCounter();
			liststream << "sketch" << std::endl;
			liststream << "id: " << (elements[i])->id << std::endl;
			liststream << "parentId: " << (elements[i])->parentId << std::endl;
			liststream << "name: " << (elements[i])->name << std::endl;
			liststream << "sketchtype: " << elements[i]->getType() << std::endl;
			liststream << "color: " << toString(elements[i]->color) << std::endl;
			liststream << "colormap: " << elements[i]->colormap << std::endl;
		}
	}
	return liststream.str();
}

template <class T>  
SketchDataRoot* SketchPool<T>::retrieveSketch(int id)
{
  for(unsigned int i = 0; i < elements.size(); i++)
    if(elements[i]->id == id)
      return elements[i];
  return NULL;
}

template <class T>
void SketchPool<T>::dumpPool() const {
  printf("%4s %2s %4s %5s %3s %2s (%s)\n",
	 "num", "rf", "id", "pid","vis","cp",
	 (getSpaceName()+"."+getName()).c_str()); 
  for(unsigned int i = 0; i < elements.size(); i++)
    printf("%4d %2d %4d %5d  %1c  %1c %s\n",
	   i, 
	   (elements[i])->refcount, 
	   (elements[i])->id,
	   (elements[i])->parentId,
	   (elements[i])->viewable ? 'y' : 'n',
	   (elements[i])->clearPending ? 'y' : 'n',
	   (elements[i])->name.c_str());
}

} // namespace

#endif
