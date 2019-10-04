#include "HelpControl.h"
#include "Shared/Config.h"
#include "Wireless/Socket.h"

//#define HelpControl_HTML_

ControlBase * HelpControl::activate(MC_ID disp_id, Socket * gui) {
	const char * fmt="  * ";
	if(config->main.use_VT100) {
		fmt="\33[1m  * \33[0m";
		sout->printf("%s","\33[1m* Global commands\33[0m: type these anytime, interpreted directly by Controller\n");
	} else
		sout->printf("%s","* Global commands: type these anytime, interpreted directly by Controller\n");
	sout->printf("%s%s",fmt,"'!refresh' - redisplays the current control (handy on first connecting, or\n     when other output has scrolled it off the screen)\n");
	sout->printf("%s%s",fmt,"'!reset' - return to the root control\n");
	sout->printf("%s%s",fmt,"'!next' - calls doNextItem() of the current control\n");
	sout->printf("%s%s",fmt,"'!prev' - calls doPrevItem() of the current control\n");
	sout->printf("%s%s",fmt,"'!select' - calls doSelect() of the current control\n");
	sout->printf("%s%s",fmt,"'!cancel' - calls doCancel() of the current control\n");
	sout->printf("%s%s",fmt,"'!dump_stack' - requests a dump of the current stack of submenus (useful if the GUI (re)connects and thus current robot state is unknown)\n");
	sout->printf("%s%s",fmt,"'!msg text' - broadcasts text as a TextMsgEvent\n");
	sout->printf("%s%s",fmt,"'!root text' - - calls ControlBase::takeInput(text) on the root control\n");
	sout->printf("%s%s",fmt,"'!hilight [n1 [n2 [...]]]' - hilights zero, one, or more items in the menu\n");
	sout->printf("%s%s",fmt,"'!input text' - calls takeInput(text) on the currently hilighted control(s)\n");
	sout->printf("%s%s",fmt,"'!set section.key=value' - will be sent to Config::setValue(section,key,value)\n");
	sout->printf("%s%s",fmt,"any text not beginning with ! - sent to takeInput() of the current control\n");
	report(root,"",maxDepth);
	return NullControl::activate(disp_id,gui);
}

//! displays the menu items of @a r and their descriptions, recursing on submenus
/*! @a prefix is what should be displayed before each menu item (like a bullet point)
 *  this is itself prefixed by 2 spaces for each level of recursion.  Word wrapping
 *  is performed to maintain the clean indenting */
void HelpControl::report(ControlBase* r, const std::string& prefix, unsigned int depth_remain) {
	if(r==NULL || depth_remain==0)
		return;
	const std::vector<ControlBase*>& slots=r->getSlots();
	const std::string pre="  "+prefix;
	unsigned int numlen=1;
	if(slots.size()>1)
		numlen=(int)(log(slots.size()-1.0)/log(10.0))+1;
#ifdef HelpControl_HTML_
	unsigned int ngoodslots=0;
	for(unsigned int i=0; i<slots.size(); i++)
		if(slots[i]!=NULL)
			ngoodslots++;
	if(ngoodslots>0)
		sout->printf("<ol>\n");
#endif
	for(unsigned int i=0; i<slots.size(); i++) {
		if(slots[i]==NULL)
			continue;
		const char * fmt;
		std::string nm=slots[i]->getName();
		std::string desc=slots[i]->getDescription();
		unsigned int len=term_width-(prefix.size()+nm.size()+4+numlen);
		if((int)len<0)
			len=0;
		if(len>desc.size())
			len=desc.size();
		else
			while(len>0 && !isspace(desc[len-1])) len--;
#ifdef HelpControl_HTML_
		fmt="%s<li value=\"%*d\"><code><b>%s</b>: %s";
#else
		if(config->main.use_VT100)
			fmt="\33[1m%s%*d. %s\33[0m: %s\n";
		else
			fmt="%s%*d. %s: %s\n";
#endif
		sout->printf(fmt,prefix.c_str(),numlen,i,nm.c_str(),desc.substr(0,len).c_str());
		while(len<desc.size() && isspace(desc[len])) len++;
		desc=desc.substr(len);
		while(desc.size()>0) {
			len=term_width-prefix.size();
			if((int)len<0)
				len=0;
			if(len>desc.size())
				len=desc.size();
			else {
				while(len>0 && !isspace(desc[len-1])) len--;
				if(len==0)
					len=term_width-prefix.size();
				if(len>desc.size())
					len=desc.size();
			}
#ifdef HelpControl_HTML_
			sout->printf("\n%s",desc.substr(0,len).c_str());
#else
			sout->printf("%s%s\n",std::string(prefix.size(),' ').c_str(),desc.substr(0,len).c_str());
#endif
			while(len<desc.size() && isspace(desc[len])) len++;
			desc=desc.substr(len);
		}
#ifdef HelpControl_HTML_
		sout->printf("</code></li>\n");
#endif
		report(slots[i],pre,depth_remain-1);
	}
#ifdef HelpControl_HTML_
	if(ngoodslots>0)
		sout->printf("</ol>\n");
#endif
}

/*! @file
 * @brief Implements HelpControl, which recurses through the menu system and outputs the name and description of each item
 * @author ejt (Creator)
 */

