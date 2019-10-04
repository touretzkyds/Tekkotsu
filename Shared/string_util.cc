#include "string_util.h"
#include <cctype>
#include <locale>
#include <pwd.h>
#include <stdlib.h>
#include <regex.h>
#include <sstream>
#include <algorithm>
#include <memory>
#ifdef __GNUC__
#  include <cxxabi.h> // demangling
#endif

using namespace std;

namespace string_util {

	//! reference to the current standard library 'locale'
	static const std::locale& curLocale=std::locale::classic();
	
	char localeToUpper(char c) {
		return std::toupper(c,curLocale);
	}

	char localeToLower(char c) {
		return std::tolower(c,curLocale);
	}
	
	string makeUpper(const string& s) {
		string ans(s); // yes, I actually checked if it's faster to copy and then overwrite or reserve and use std::back_inserter(ans)
		std::transform(ans.begin(), ans.end(), ans.begin(), (int(*)(int)) std::toupper);
		return ans;
	}
	
	string makeLower(const string& s) {
		string ans(s);
		std::transform(ans.begin(), ans.end(), ans.begin(), (int(*)(int)) std::tolower);
		return ans;
	}

	string removePrefix(const string& str, const string& pre) {
		if(str.compare(0,pre.size(),pre)==0)
			return str.substr(pre.size());
		return string();
	}
	
	string tildeExpansion(const string& str) {
		string ans;
		if(str[0]!='~') {
			return str;
		}
#ifndef PLATFORM_APERIOS
		else if(str=="~" || str[1]=='/') {
			char* home=getenv("HOME");
			if(home==NULL)
				return str;
			if(str=="~")
				return home;
			return home+str.substr(1);
		} else {
			string::size_type p=str.find('/');
			struct passwd * pw;
			pw=getpwnam(str.substr(1,p-1).c_str());
			if(pw==NULL)
				return str;
			return pw->pw_dir+str.substr(p);
		}
#else
		return str.substr(1);
#endif
	}

	string trim(const string& str) {
		if(str.size()==0)
			return str;
		unsigned int b=0;
		unsigned int e=str.size()-1;
		while(b<str.size() && isspace(str[b]))
			b++;
		while(b<e && isspace(str[e]))
			e--;
		return str.substr(b,e-b+1);
	}
	
	bool beginsWith(const std::string& str, const std::string& prefix) {
		if(str.size()<prefix.size()) // str is shorter than prefix, cannot contain suffix
			return false;
		if(prefix.size()==0) // empty suffix matches everything
			return true;
		return str.compare(0,prefix.size(),prefix)==0;
	}
	
	bool endsWith(const std::string& str, const std::string& suffix) {
		if(str.size()<suffix.size()) // str is shorter than suffix, cannot contain suffix
			return false;
		if(suffix.size()==0) // empty suffix matches everything
			return true;
		return str.compare(str.size()-suffix.size(),suffix.size(),suffix)==0;
	}
	
	std::string makePath(const std::string& rel, const std::string& path) {
		if(path[0]=='/') {
			return path;
		} else {
			std::string::size_type d = rel.rfind('/');
			if(d==std::string::npos) {
				return path;
			} else {
				return rel.substr(0,d+1)+path;
			}
		}
	}
	
	std::vector<std::string> tokenize(const std::string & str, const std::string & delims /*=", \t"*/) {
		// Skip delims at beginning, find start of first token
		string::size_type lastPos = str.find_first_not_of(delims, 0);
		// Find next delimiter at end of token
		string::size_type pos     = str.find_first_of(delims, lastPos);
		
		vector<string> tokens; // output vector
		while (string::npos != pos || string::npos != lastPos) {
			// Found a token, add it to the vector.
			tokens.push_back(str.substr(lastPos, pos - lastPos));
			// Skip delims.  Note the "not_of". this is beginning of token
			lastPos = str.find_first_not_of(delims, pos);
			// Find next delimiter at end of token.
			pos     = str.find_first_of(delims, lastPos);
		}
		return tokens;
	}
	
	bool parseArgs(const string& input, vector<string>& args, vector<unsigned int>& offsets) {
		string cur;
		bool isDoubleQuote=false;
		bool isSingleQuote=false;
		args.clear();
		offsets.clear();
		unsigned int begin=-1U;
		for(unsigned int i=0; i<input.size(); i++) {
			char c=input[i];
			if(begin==-1U && !isspace(c))
				begin=i;
			switch(c) {
			case ' ':
			case '\n':
			case '\r':
			case '\t':
			case '\v':
			case '\f':
				if(isSingleQuote || isDoubleQuote)
					cur+=c;
				else if(cur.size()!=0) {
					args.push_back(cur);
					offsets.push_back(begin);
					cur.clear();
					begin=-1U;
				}
				break;
			case '\\':
				if(i==input.size()-1) { //escaped line break
					return false;
				} else
					cur.push_back(input[++i]);
				break;
			case '"':
				if(isSingleQuote)
					cur.push_back(c);
				else
					isDoubleQuote=!isDoubleQuote;
				break;
			case '\'':
				if(isDoubleQuote)
					cur+=c;
				else
					isSingleQuote=!isSingleQuote;
				break;
			default:
				cur+=c;
				break;
			}
		}
		if(cur.size()>0) {
			args.push_back(cur);
			offsets.push_back(begin);
		}
		return !isDoubleQuote && !isSingleQuote;
	}

	bool reMatch(const std::string& str, const std::string& regex) {
		return reMatch(str,regex,REG_EXTENDED);
	}
	
	bool reMatch(const std::string& str, const std::string& regex, int flags) {
		regex_t re;
		if(int err=regcomp(&re,regex.c_str(),flags | REG_NOSUB)) {
			char msg[128];
			regerror(err,&re,msg,128);
			string errmsg;
			errmsg.append("Bad filter '").append(regex).append("': ").append(msg);
			regfree(&re);
			throw errmsg;
		}
		int match=regexec(&re,str.c_str(),0,NULL,0);
		regfree(&re);
		if(match==0) {
			return true;
		} else if(match==REG_NOMATCH) {
			return false;
		} else {
			char msg[128];
			regerror(match,&re,msg,128);
			string errmsg;
			errmsg.append("Regex error on reMatch('").append(str).append("', '").append(regex).append("'): ").append(msg);
			throw errmsg;
		}
	}

	std::string intToStringIP(int ip) {
		std::stringstream ret;
		ret << (ip >> 24 & 0xff) << '.' << (ip >> 16 & 0xff) << '.'
		<< (ip >> 8 & 0xff) << '.' << (ip >> 0 & 0xff);
		return ret.str();
	}
	
	int stringToIntIP(std::string ip_str) {
		std::istringstream sstr;
		sstr.str(ip_str);
		int ip = 0, b;
		char c;
		
		sstr >> b >> c;
		ip |= b << 24;
		
		sstr >> b >> c;
		ip |= b << 16;
		
		sstr >> b >> c;
		ip |= b << 8;
		
		sstr >> b;
		ip |= b << 0;
		
		return ip;
	}
	
	size_t utf8len(const std::string& str) {
		size_t len=0, i=0;
		while(i<str.size()) {
			unsigned char c = str[i];
			if(c < 0x80)
				++i;
			else if((c >> 5) == 0x6)
				i+=2;
			else if((c >> 4) == 0xe)
				i+=3;
			else if((c >> 3) == 0x1e)
				i+=4;
			else
				++i; // actually invalid, but we're not checking for that, so count as one
			++len;
		}
		return len;
	}

#ifndef __GNUC__
	std::string demangle(const std::string& symbol) { return symbol; }
#else
	std::string demangle(const std::string& symbol) {
		std::string ret = symbol;
		try {
			int status = 0;
			std::auto_ptr<const char> d(abi::__cxa_demangle(symbol.c_str(), 0, 0, &status));
			if(d.get()!=NULL)
				ret = d.get();
		} catch(...) {  }
		return ret;
	}
#endif
	
	std::string int2str(int const n) {
		ostringstream os;
		os << n;
		return os.str();
	}
	
}

/*! @file
 * @brief Implements some useful functions for string manipulation in the string_util namespace
 * @author ejt (Creator)
 */
