#ifndef MODULE_NNIRD0YI
#define MODULE_NNIRD0YI

#include <string>
#include <vector>
#include <iostream>
#include <memory/MemoryBlock.h>
#include <memory/Memory.h>

class Module {
public:
    Module ();

    void init(Memory *memory);

protected:
    void requiresMemoryBlock(const std::string &name);
    virtual void specifyMemoryBlocks() = 0;

    Memory *memory_; // Should go private once we remove bhuman walk module

protected:
    template <class T>
    void getMemoryBlock(T *&ptr, const std::string &name) {
        for (unsigned int i = 0; i < required_blocks_.size(); i++) {
            if (required_blocks_[i] == name) {
                memory_->getBlockByName(ptr,name);
                return;
            }
        }
        std::cerr << "Module::getMemoryBlock - ERROR: tried to get a memory block that was not required: " << name << std::endl;
        return;
    }

    template <class T>
    void getOrAddMemoryBlock(T *&ptr, const std::string &name) {
        for (unsigned int i = 0; i < required_blocks_.size(); i++) {
            if (required_blocks_[i] == name) {
                memory_->getOrAddBlockByName(ptr,name);
                return;
            }
        }
        std::cerr << "Module::getMemoryBlock - ERROR: tried to get a memory block that was not required: " << name << std::endl;
        return;
    }

private:
    std::vector<std::string> required_blocks_;
};

#endif /* end of include guard: MODULE_NNIRD0YI */
