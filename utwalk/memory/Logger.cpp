#include "Logger.h"
#include <iostream>

Logger::Logger(const char filename[]) {
    //log_file_.open(filename,std::ios::binary);
    log_file_.open(filename);
}

Logger::~Logger() {
    close();
}

void Logger::writeMemoryHeader(const MemoryHeader &header) {
    unsigned int num_blocks = header.block_names.size();
    log_file_.write((const char*)&(num_blocks),sizeof(num_blocks));
    // write each block name
    for (unsigned int i = 0; i < num_blocks; i++)
        log_file_.write(header.block_names[i].c_str(),header.block_names[i].size()+1); // +1 so we write the \0
}

void Logger::writeMemory(const Memory &memory) {
    MemoryHeader header;
    // first get a list of the blocks we're loading
    memory.getBlockNames(header.block_names,true);
    // write the header
    writeMemoryHeader(header);
    // write each block
    for (unsigned int i = 0; i < header.block_names.size(); i++) {
        writeBlock(*(memory.getBlockPtrByName(header.block_names[i])));
    }
}

void Logger::writeBlock(const MemoryBlock &block) {
    log_file_.write((const char *)&block.header,sizeof(block.header)); // write out the header
    log_file_.write((const char *)&block,block.header.size); // write the block
}

void Logger::close() {
    log_file_.close();
}
