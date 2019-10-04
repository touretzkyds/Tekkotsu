#include "FileBrowserControl.h"
#include "NullControl.h"
#include "Shared/Config.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

using namespace std;

ControlBase * FileBrowserControl::activate(MC_ID display, Socket * gui) {
	rebuildmenu();
	return ControlBase::activate(display,gui);
}

ControlBase* FileBrowserControl::doSelect() {
	for(unsigned int i=0; i<hilights.size(); i++) {
		unsigned int cur=hilights[i];
		if(cur>=options.size() || options[cur]==NULL)
			continue;
		ControlBase::doSelect();
		std::string nm(options[cur]->getName());
		if(nm[nm.size()-1]=='/' || nm=="..") {
			if(hilights.size()>1)
				continue;
			if(nm=="..")
				paths.pop_back();
			else
				paths.push_back(nm.substr(0,nm.size()-1));
			rebuildmenu();
			refresh();
			return this;
		} else {
			ControlBase * ret=selectedFile(makePath(nm));
			if(ret!=this)
				return ret;
		}
	}
	refresh();
	return this;
}

ControlBase * FileBrowserControl::takeInput(const std::string& msg) {
	if(msg.size()==0)
		return this;
	if(msg.find('/')==string::npos) {
		if(options.size()==1 && options.front()==NULL)
			rebuildmenu();
		return ControlBase::takeInput(msg);
	} else {
		string::size_type pos=msg.rfind('/');
		if(msg[0]=='/') {
			if(msg.substr(0,root.size())!=root)
				return this;
			if(msg.size()>root.size() && msg[root.size()]!='/')
				return this;
			paths.clear();
			if(pos<=root.size())
				return this;
			appendPath(msg.substr(root.size(),pos-root.size()));
		} else {
			appendPath(msg.substr(0,pos));
		}
		rebuildmenu();
		if(msg.size()>pos+1)
			return ControlBase::takeInput(msg.substr(pos+1));
		return this;
	}
}

void FileBrowserControl::setRoot(const std::string& path) {
	root=config->portPath(path);
	if(root[root.size()-1]=='/')
		root.erase(root.size()-1);
	paths.clear();
}


void FileBrowserControl::appendPath(const std::string& path) {
	paths.push_back(std::string());
	for(unsigned int i=0; i<path.size(); i++) {
		if(path[i]!='/')
			paths.back().append(1,path[i]);
		else if(paths.back().size()!=0)
			paths.push_back(std::string());
	}
}


std::string FileBrowserControl::makePath() {
	std::string path=root;
	for(unsigned int i=0; i<paths.size(); i++) {
		path+="/";
		path+=paths[i];
	}
	return path;
}

//! returns the path from root as a string, appends filename
std::string FileBrowserControl::makePath(const std::string& filename) {
	std::string path=makePath();
	path.append("/");
	path.append(filename);
	return path;
}
	
bool FileBrowserControl::match(const std::string& file, const std::string& filt) {
	unsigned int i=0;
	if(i==filt.size() && i==file.size())
		return true;
	if(i==filt.size() || i==file.size())
		return false;
	while(filt[i]!='*') {
		if(toupper(filt[i])!=toupper(file[i]))
			return false;
		i++;
		if(i==filt.size() && i==file.size())
			return true;
		if(i==filt.size() || i==file.size())
			return false;
	}
	i=filt.size()-1;
	unsigned int j=file.size()-1;
	while(filt[i]!='*') {
		if(toupper(filt[i])!=toupper(file[j]))
			return false;
		i--; j--;
	}
	return true;
}

void FileBrowserControl::rebuildmenu() {
	clearSlots();
	DIR* dir=opendir(makePath().c_str());
	if(dir==NULL) {
		pushSlot(new NullControl("Bad Path: "+makePath(),makePath(),this));
		return;
	}
	if(paths.size()!=0 && recurse) {
		struct stat s;
		std::string path=makePath("..");
		int err=stat(path.c_str(),&s);
		if(err==0 && s.st_mode&S_IFDIR)
			pushSlot(new NullControl("..","go up a directory level",this));
	}
	std::map<std::string,std::string> files;
	struct dirent * ent=readdir(dir);
	while(ent!=NULL) {
		if(strcmp(ent->d_name,".")!=0 && strcmp(ent->d_name,"..")!=0) {
			struct stat s;
			std::string path=(makePath(ent->d_name));
			int err=stat(path.c_str(),&s);
			if(err!=0) {
				cout << "File disappeared: " << path << endl;
				return;
			}
			if(s.st_mode&S_IFDIR) {
				if(recurse)
					files[std::string(ent->d_name).append(1,'/')] = makePath(ent->d_name);
			} else {
				std::string nm=(makePath(ent->d_name));
				if(match(nm,filter))
					files[ent->d_name] = makePath(ent->d_name);
			}
		}
		ent=readdir(dir);
	}
	closedir(dir);
	for(std::map<std::string,std::string>::const_iterator fit=files.begin(); fit!=files.end(); ++fit)
		pushSlot(new NullControl(fit->first,fit->second,this));
	if(options.size()==0)
		pushSlot(new NullControl("[empty directory]",makePath(),this));
	else {
		hilights.push_back(0);
		/*	for(unsigned int i=0; i<hilights.size(); i++) {
				if(hilights[i]>=options.size()) {
				hilights.resize(i);
				cout << "hilights resized at " << i << endl;
				break;
				}
				}*/
	}
}


/*! @file
 * @brief Implements FileBrowserControl, which displays the contents of a directory
 * @author ejt (Creator)
 */

