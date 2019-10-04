//-*-c++-*-
#ifndef INCLUDED_SpeechNode_h_
#define INCLUDED_SpeechNode_h_

#include <sstream>   // for ostringstream

#include "Behaviors/StateNode.h"
#include "Sound/SoundManager.h"

//! A StateNode that speaks text upon startup and posts a status event on completion.
/*! Speech text comes from three places: the @a text argument, a DataEvent<string>,
 *  and anything the user writes to #textstream, in that order.
 *  By default, sound playback will continue even after this node has been deactivated.
 *  If this is not the desired behavior, set the #autostop flag (through setAutoStop())
 */
class SpeechNode : public StateNode {
public:
  //! constructor, specify instance name and string to speak
  SpeechNode(const std::string& nodename, const std::string& text="")
    : StateNode(nodename), storedText(text), textstream(std::ios_base::ate),
      showText(true), curplay_id(SoundManager::invalid_Play_ID), autostop(false),
      savedText(), pos() {}
  
  //! activate the node, starts playing the text
  virtual void preStart() {
    StateNode::preStart();
    
    // Speech text comes from three places: the storedText variable, a DataEvent<string>,
    // and anything the user writes to textstream.  We concatenate them in that order.
    textstream.str(storedText);
    if ( event != NULL ) {
      const DataEvent<std::string> *datev = dynamic_cast<const DataEvent<std::string>*>(event);
      if ( datev != NULL )
        textstream << (storedText.empty() ? "" : " ") << datev->getData();
    }
    savedText = textstream.str();
    textstream << (textstream.str().empty() ? "" : " ");  // add a trailing space in case user adds text
    pos = textstream.tellp();
  }
  
  virtual void postStart() {
    StateNode::postStart();
    if ( textstream.tellp() == pos )
      textstream.str(savedText);  // get rid of the trailing space if user didn't add text
    if ( textstream.str().empty() )
      textstream << "Speech node " + getName() + " has no text to speak";
    curplay_id = sndman->speak(textstream.str(), showText);
    if ( curplay_id == SoundManager::invalid_Play_ID ) // something is broken, so punt
      postStateCompletion();
    else 
      erouter->addListener(this, EventBase::audioEGID, curplay_id, EventBase::deactivateETID);
  }
  
  //! deactivate the node, doesn't stop the sound playback unless the #autostop flag has been set
  virtual void stop() {
    if ( autostop )
      stopPlay();
    StateNode::stop();
  }
  
  //! receive audioEGID status event and post stateMachineEGID status event
  virtual void doEvent() {
    curplay_id = SoundManager::invalid_Play_ID;
    postStateCompletion();
  }
  
  //! interrupts playing of the current sound
  void stopPlay() {
    sndman->stopPlay(curplay_id);
    curplay_id = SoundManager::invalid_Play_ID;
  }
  
  //! returns the stored text associated with this node
  const std::string getText() const { return storedText; }
  
  //! sets the stored text associated with this node
  void setText(const std::string &text) { storedText = text; }
  
  //! controls whether the text will be printed as well as spoken
  void setShowText(bool s) { showText = s; }
  
  //! returns the current status of the #autostop flag
  bool getAutoStop() const { return autostop; }
  
  //! sets the #autostop flag
  void setAutoStop(bool astop) { autostop=astop; }
  
protected:
  std::string storedText; //!< text to speak, accessed through setText() and getText()
  std::ostringstream textstream; //!< An ostringstream which the doStart can manipulate (normally by adding text)
  bool showText; //!< flag controlling whether spoken text should also be displayed on the console
  SoundManager::Play_ID curplay_id; //!< holds the playback identification so it can be halted any time
  bool autostop; //!< if set to true by setAutoStop(), when this node is deactivated, playback will be halted.  Otherwise, playback will continue even after the node is deactivated
  
private:
  std::string savedText; //!< caches text between preStart() and postStart()
  std::streampos pos; //!< caches stream size between preStart() and postStart()
};

/*! @file
 * @brief Defines SpeechNode, a simple StateNode that speaks a string and posts a status event upon completion
 * @author dst (Creator)
 */

#endif
