#include "Module.h"

Module::Module():
    memory_(NULL)
{}

void Module::init(Memory *memory) {
    memory_ = memory;
    specifyMemoryBlocks();
}

void Module::requiresMemoryBlock(const std::string &name) {
    //I commented this check out as it doesn't work well with the new getOrAdd commands (MQ 3/15/2011)

    //MemoryBlock *block;
    //memory_->getBlockByName(block,name);
    //if (block == NULL)
    //  std::cerr << "Module::requiresMemoryBlock: ERROR: requiring memory block that does not exist: " << name << std::endl;
    required_blocks_.push_back(name);
}
