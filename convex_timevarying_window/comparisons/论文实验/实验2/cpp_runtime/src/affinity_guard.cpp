#include <fstream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <chrono>
#include <sched.h>
#include <unistd.h>
#include <signal.h>
int main(int argc,char** argv) {
    if(argc!=3)return 1;
    const int coordinator=std::stoi(argv[1]);std::ofstream log(argv[2]);std::map<int,int> slots;
    while(kill(coordinator,0)==0) {
        std::ifstream children("/proc/"+std::to_string(coordinator)+"/task/"+std::to_string(coordinator)+"/children");
        std::set<int> active;int pid;while(children>>pid)active.insert(pid);
        for(auto entry=slots.begin();entry!=slots.end();) {
            if(!active.count(entry->first))entry=slots.erase(entry);else ++entry;
        }
        std::set<int> used;for(const auto& entry:slots)used.insert(entry.second);
        for(int child:active) {
            if(slots.count(child))continue;
            int cpu=0;while(used.count(cpu))cpu+=2;
            cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cpu,&mask);
            if(sched_setaffinity(child,sizeof(mask),&mask)==0) {
                slots[child]=cpu;used.insert(cpu);log<<child<<','<<cpu<<std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
