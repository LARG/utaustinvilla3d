#include "PrivateMemory.h"
#include <iostream>

PrivateMemory::PrivateMemory() {
    //std::cout << "PRIVATE MEMORY CONSTRUCTOR" << std::endl << std::flush;
}

PrivateMemory::~PrivateMemory() {
    //std::cout << "PRIVATE MEMORY DESTRUCTOR" << std::endl << std::flush;
    // loop through the blocks and destruct them if they exist

    // For some reason this delete seg faults ?
    for (MemMap::iterator it = blocks_.begin(); it != blocks_.end(); it ++) {
        //  delete (*it).second;
    }
    blocks_.clear();
}

bool PrivateMemory::addBlock(const std::string &name,MemoryBlock *block) {
    std::pair<MemMap::iterator,bool> res = blocks_.insert(std::pair<std::string,MemoryBlock*>(name,block));
    // check if the block was inserted
    if (!res.second) {
        std::cerr << "PrivateMemory::addBlock - ERROR ADDING BLOCK " << name << " already exists, Deleting given block" << std::endl;
        delete block;
    }
    return res.second;
}

MemoryBlock* PrivateMemory::getBlockPtr(const std::string &name) {
    MemMap::iterator it = blocks_.find(name);
    if (it == blocks_.end()) // the block isn't in our map
        return NULL;
    else
        return (*it).second;
}

const MemoryBlock* PrivateMemory::getBlockPtr(const std::string &name) const {
    MemMap::const_iterator it = blocks_.find(name);
    if (it == blocks_.end()) // the block isn't in our map
        return NULL;
    else
        return (*it).second;
}

void PrivateMemory::getBlockNames(std::vector<std::string> &module_names, bool only_log) const {
    for (MemMap::const_iterator it = blocks_.begin(); it != blocks_.end(); it++) {
        if ((!only_log) || (*it).second->log_block)
            module_names.push_back((*it).first);
    }
}
