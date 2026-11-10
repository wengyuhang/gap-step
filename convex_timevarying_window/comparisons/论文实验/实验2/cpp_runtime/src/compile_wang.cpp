#include <casadi/casadi.hpp>
#include "process.hpp"
#include <filesystem>
#include <iostream>
int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: compile_wang GRAPH_DIRECTORY OUTPUT_DIRECTORY GATE_INDEX");
        namespace fs=std::filesystem;
        const fs::path source=fs::absolute(argv[1]),output=fs::absolute(argv[2]);
        fs::create_directories(output);const std::string gate=argv[3],name="fg_"+gate;
        const auto function=casadi::Function::load((source/(name+".casadi")).string());
        casadi::CodeGenerator generator(name+".c");generator.add(function);generator.generate(output.string()+"/");
        run_process({"gcc","-O3","-fPIC","-shared",(output/(name+".c")).string(),"-o",(output/(name+".so")).string(),"-lm"});
        std::cout<<"COMPILED "<<source.filename()<<' '<<gate<<std::endl;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
