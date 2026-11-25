#include "mesh.hpp"
int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: mesh_check TRAJECTORY_JSON SCENE_JSON OUTPUT_JSON");
        Json input,scene;std::ifstream(argv[1])>>input;std::ifstream(argv[2])>>scene;
        const auto result=mesh_evaluate(Trajectory::load(input),scene);
        std::ofstream(argv[3])<<result.dump(2)<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
