#include "src\common_function\logger\logger.h"

int main(int argc,char* argv[]){
    LOG_INFO("project ToraDynamics demo~");
    LOG_WARNING("warning something...");
    LOG_DEBUG("debug something...");
    if (false){
        LOG_ERROR("error something...");
    }
    ASSERT(1==1);
    ASSERT(1==2);
    return 0;
}