#ifdef DOCDUMP

#define INCLUDED_PListSupport_h_
#define INCLUDED_plistSpecialty_h_

#define INCLUDED_SensorInfo_h_
struct SensorInfo {};

#include <string>
#include <map>
#include <set>
#include <typeinfo>
#include <iostream>
#include <sstream>
#include "Shared/string_util.h"
struct xmlNode;

using namespace std;

struct Row {
	
	Row(const string& n, const string& t, const string& d) : name(n), type(t), description(d) {}
	string name;
	string type;
	string description;
	bool operator<(const Row& r) const { return name < r.name; }
};
ostream& operator<<(ostream& os, const Row& r) {
	string desc = r.description;
	for(string::size_type p=desc.find('\n'); p!=string::npos; p=desc.find('\n'))
		desc.replace(p,1,"<br>");
	os << "\t\t<tr> <td>" << r.name << "</td> <td>" << r.type << "</td> <td>" << desc << "</td> </tr>";
	return os;
}
typedef set<Row> Table;
typedef map<string,Table > DB;
DB db;

string readableTypeName(const type_info& ti) {
	if(ti==typeid(bool))
		return "bool";
	else if(ti==typeid(float))
		return "float";
	else if(ti==typeid(unsigned int))
		return "unsigned int";
	else if(ti==typeid(int))
		return "int";
	else if(ti==typeid(string))
		return "string";
	else {
		string t=ti.name();
		if(t[0]=='N' || isdigit(t[0])) {
			string type = string_util::tokenize(t,"1234567890").back();
			if(t[0]=='N')
				type.resize(type.size()-1);
			return type;
		}
		return t;
	}
}
template<class T> string readableTypeName() { return readableTypeName(typeid(T)); }

namespace plist {
	class Listener {};
	class PrimitiveListener : public Listener {};
	class CollectionListener : public Listener {};
	struct ObjectBase {
		virtual ~ObjectBase() {}
	};
	struct Collection : public ObjectBase {
		void addCollectionListener(CollectionListener*) const {}
		void removeCollectionListener(CollectionListener*) const {}
	};
	class Dictionary : public Collection {
	public:
		typedef pair<string,ObjectBase*>* const_iterator;
		string getTypeName() const {
			string name = readableTypeName(typeid(*this));
			return "<a href=\"#"+name+"Params\">"+name+"</a>";
		}
		virtual ~Dictionary() {}
		virtual void saveXML() {}
		const_iterator findEntry(const string& x) const { return 0; }
		const_iterator begin() const { return 0; }
		const_iterator end() const { return 0; }
	protected:
		void saveXMLNode(xmlNode*,string,ObjectBase*,string,int) const {}
		template<class T> void addEntry(const string& name, T& val, const string& desc="") {
			db[readableTypeName(typeid(*this))].insert(Row(name, val.getTypeName(), desc));
		}
		enum LoadSavePolicy_t { FIXED,SYNC };
		void setLoadSavePolicy(LoadSavePolicy_t l, LoadSavePolicy_t s) {}
	};
	template<class T> struct DictionaryOf : public Collection {
	typedef pair<string,ObjectBase*>* const_iterator;
		static string getTypeName() {
			string name = readableTypeName<T>();
			return "Dictionary of <a href=\"#"+name+"Params\">"+name+"s</a>";
		}
	};
	struct Array : public Collection {
		void addEntry(ObjectBase&) {}
		void addEntry(ObjectBase*) {}
	};
	template<class T> struct ArrayOf : public Collection {
		typedef T** const_iterator;
		static string getTypeName() {
			return "<span style=\"font-style: italic\">"+readableTypeName<T>()+"</span> array";
		}
		const_iterator begin() const { return 0; }
		const_iterator end() const { return 0; }
		void addEntry(T&) {}
		void addEntry(T*) {}
		unsigned int loadXML(xmlNode*) { return 0; }
	};
	struct PrimitiveBase : public ObjectBase {};
	template<typename T> struct Primitive : public PrimitiveBase {
		static string getTypeName() { return readableTypeName<T>(); }
		Primitive() {}
		Primitive(const T& v) {}
		operator T() const { return T(); }
	};
	template<typename T> struct NamedEnumeration {
		static string getTypeName() { return readableTypeName<T>()+string(" enumeration"); }
		//NamedEnumeration() : names(NULL) {}
		NamedEnumeration(T, const char* const n[]) : names() {
			for(unsigned int i=0; n && n[i]; ++i) {
				names[static_cast<T>(i)]=n[i];
			}
		}
		void addNameForVal(const string&, T ) { }
		void setPreferredNameForVal(const string& name, T v) { names[static_cast<T>(v)]=name; }
		string getDescription() const {
			string s;
			for(typename std::map<T,string>::const_iterator it=names.begin(); it!=names.end(); ++it) {
				if(s.size()!=0)
					s += " | ";
				s += it->second;
			}
			return "Value is one of: { " + s + "}";
		}
		map<T,string> names;
	};
	template<typename T> struct RGBColor {
		static string getTypeName() { return "color"; }
		RGBColor() {}
		RGBColor(T, T, T) {}
	};
	struct Point {
		static string getTypeName() { return "float[3]"; }
		Point() {}
		Point(float, float, float) {}
		float operator[](size_t i) const { return 0; }
	};
	template<class T> struct PrimitiveCallbackMember {
		template<class P, class C> PrimitiveCallbackMember(const P&, const T&, const C&, bool call=false) {}
	};
	template<class T> struct CollectionCallbackMember {
		template<class P, class C> CollectionCallbackMember(const P&, const T&, const C&, bool call=false) {}
	};
	typedef Primitive<float> Angle;
	struct OutputSelector : Primitive<string> {
		static const unsigned int UNUSED=-1U;
		operator int() const { return 0; }
		string get() const { return ""; }
		void setRange(size_t a, size_t b) {}
	};
	#define INSTANTIATE_NAMEDENUMERATION_STATICS(x)
	#define PLIST_CLONE_DEF(a,b)
}

#include "EnvConfig.h"

void LinkComponent::updateBB() const {}
void LinkComponent::dirtyBB() {}
void LinkComponent::sumLinkCenterOfMass(fmat::Column<3ul, float>&, float&) const {}

namespace EnvConfig {

	const char* const Background::modelNames[5] = { "", "Plane", "Box", "Dome", NULL };
	INSTANTIATE_NAMEDENUMERATION_STATICS(EnvConfig::Background::model_t);
	
	Environment& singleton() {
		static Environment env;
		return env;
	}
	
	void Environment::resetDefaults() {}
}
void PhysicsWorld::initWorld() {}
void PhysicsWorld::updateGravity() {}
void PhysicsWorld::updateSolverInfo() {}
PhysicsWorld::~PhysicsWorld() {}
Physics* Physics::inst=NULL;

void mergeTable(const string& n1, const string& n2, const string& prefix="") {
	for(Table::const_iterator it=db[n1].begin(); it!=db[n1].end(); ++it) {
		Row r(*it);
		r.name=prefix+r.name;
		db[n2].insert(r);
	}
}

void displayTable(const string& name, const Table t) {
	cout << "<a name=\"" << name << "Params\"></a><table class=\"borders ParamList\" border=\"0\" cellspacing=\"0\" cellpadding=\"0\">\n";
	cout << "\t<caption>" << name << " Parameters</caption>\n";
	cout << "\t<thead><tr><th>Name</th><th>Type</th><th>Description</th></tr></thead>\n";
	cout << "\t<tbody>\n";
	for(Table::const_iterator rit=t.begin(); rit!=t.end(); ++rit) {
		cout << *rit << '\n';
	}
	cout << "\t</tbody>\n";
	cout << "</table>\n";
}

void manualDisplay(const string& s, set<string>& prev) {
	displayTable(s,db[s]);
	prev.insert(s);
}

int main(int argc, const char* const argv[]) {
	cout << "<!-- > Generated by Tekkotsu/tools/mirage/docdump, build via 'make docdump' -->\n";
	EnvConfig::singleton();
	EnvConfig::Light();
	EnvConfig::PhysicalObject();
	
	mergeTable("Object","PhysicalObject","(Object) ");
	mergeTable("Object","Light","(Object) ");
	mergeTable("PhysicsWorld","Physics");
	db.erase("PhysicsWorld");
	db["PhysicalObject"].insert(Row("(LinkComponent)","*","Inherits all <a href=\"#LinkComponentParams\">LinkComponent</a> fields: for a single component, just describe directly at the root, otherwise add to Components list (or use Kinematics to load a file)."));
	
	set<string> manualOrder;
	manualDisplay("Environment",manualOrder);
	manualDisplay("Background",manualOrder);
	manualDisplay("Object",manualOrder);
	for(DB::const_iterator tit=db.begin(); tit!=db.end(); ++tit) {
		if(manualOrder.count(tit->first)>0)
			continue;
		displayTable(tit->first,tit->second);
	}
	return EXIT_SUCCESS;
}

#endif
