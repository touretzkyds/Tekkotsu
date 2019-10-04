#include "Wireless/netstream.h"
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>

using namespace std;

sem_t* done;
int verbose=0;

void thread_setup() {
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
}

void* reader(void* p) {
	thread_setup();
	string x;
	ionetstream& io=*(ionetstream*)p;
	getline(io,x);
	while(io || io.getReconnect()) {
		cout << x << endl;
		io.clear();
		getline(io,x);
		pthread_testcancel();
	}
	//cout << "Read posting done" << endl;
	if(sem_post(done)!=0) {
		perror("sem_post");
		exit(1);
	}
	//cout << "Read done" << endl;
	return NULL;
}

void* writer(void* p) {
	thread_setup();
	string x;
	ionetstream& io=*(ionetstream*)p;
	getline(cin,x);
	while(cin) {
		io << x << endl;
		if(!io) {
			if(verbose)
				cerr << "Remote closed" << endl;
			if(!io.getReconnect())
				break;
			io.clear();
			//io.seekp(0); // to avoid buffering when closed...
		}
		pthread_testcancel();
		getline(cin,x);
	}
	//cout << "Write posting done" << endl;
	if(sem_post(done)!=0) {
		perror("sem_post");
		exit(1);
	}
	//cout << "Write done" << endl;
	return NULL;
}

void test_cancel(int) { pthread_testcancel(); }

int main(int argc, const char* argv[]) {
	std::ios::sync_with_stdio(false);  // optional, only affects cout, etc.
	
	IPaddr a;
	IPaddr::ipport_t localport=0;
	bool datagram=false;
	bool server=false;
	bool reconnect=false;
	int i=1;
	for(; i<argc; i++) {
		if(argv[i][0]!='-')
			break;
		else {
			string flags = argv[i]+1;
			for(string::size_type j=0; j<flags.size(); ++j) {
				if(flags[j]=='u') { datagram=true; }
				else if(flags[j]=='l') { server=true; }
				else if(flags[j]=='p' && i+1<argc) { localport=atoi(argv[++i]); }
				else if(flags[j]=='v') { verbose++; }
				else if(flags[j]=='r') { reconnect=true; }
				else {
					cerr << "unknown flag " << flags[j] << endl;
					return 2;
				}
			}
		}
	}
	if(i<argc)
		a.set_name(argv[i++]);
	if(i<argc) {
		int p=atoi(argv[i++]);
		a.set_port(p);
	}
	//cout << datagram << ' ' << server << endl;
	ionetstream io;
	io.setReconnect(reconnect);
	if(server) {
		if(verbose)
			cout << "Listening on port " << localport << "..." << flush;
		io.listen(localport,datagram);
	} else {
		if(verbose)
			cout << "Connecting to " << a.get_rname() << ' ' << a.get_display_num() << ' ' << a.get_port() << "..." << flush;
		io.open(a,datagram);
	}
	io.setEcho(verbose>1);
	if(verbose)
		cout << "connected to " << io.getPeerAddress().get_display_num() << ":" << io.getPeerAddress().get_port()
			<< " from " << io.getLocalAddress().get_display_num() << ":" << io.getLocalAddress().get_port() << endl;
	string x;
	if(!io.is_open())
		return 1;
	done=sem_open("done",O_CREAT|O_EXCL,0777,0);
	if((size_t)done==(size_t)SEM_FAILED) {
		perror("sem_open");
		if(sem_unlink("done")!=0) {
			perror("sem_unlink");
			return 1;
		}
		return 1;
	}
	if(sem_unlink("done")!=0) {
		perror("sem_unlink");
		return 1;
	}
	pthread_t rt;
	pthread_create(&rt,NULL,reader,&io);
	pthread_t wt;
	pthread_create(&wt,NULL,writer,&io);
	
	do {
		if(sem_wait(done)!=0 && errno!=EINTR) {
			perror("sem_wait");
			return 1;
		}
	} while(errno==EINTR);
	
	//cout << "Cancelling..." << endl;
	pthread_cancel(rt);
	pthread_cancel(wt);
	signal(SIGUSR1,test_cancel);
	pthread_kill(rt,SIGUSR1);
	pthread_kill(wt,SIGUSR1);
	//cout << "Joining read..." << endl;
	pthread_join(rt,NULL);
	//cout << "Joining write..." << endl;
	pthread_join(wt,NULL);
	pthread_detach(rt);
	pthread_detach(wt);
	if(sem_close(done)!=0) {
		perror("sem_close");
		return 1;
	}
	return 0;
}
