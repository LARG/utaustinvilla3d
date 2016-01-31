#ifndef LOGGER_3AB7OTVI
#define LOGGER_3AB7OTVI

#include <fstream>
#include "Memory.h"
#include "MemoryBlock.h"

class Logger {
public:
    Logger (const char filename[]);
    virtual ~Logger ();

    void writeMemoryHeader(const MemoryHeader &header);
    void writeMemory(const Memory &memory);
    void writeBlock(const MemoryBlock &block);
    void close();

private:
    std::ofstream log_file_;
};

#endif /* end of include guard: LOGGER_3AB7OTVI */
