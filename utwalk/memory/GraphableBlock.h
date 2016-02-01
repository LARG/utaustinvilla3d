#ifndef GRAPHABLEBLOCK_8VXFI3H7
#define GRAPHABLEBLOCK_8VXFI3H7

#include "MemoryBlock.h"
#include <cstring>

#define NUM_GRAPHABLE_DATA 100
#define MAX_NAME_LENGTH 30

struct GraphableBlock : public MemoryBlock {
public:
    GraphableBlock():
        size_(0)
    {
        header.version = 0;
        header.size = sizeof(GraphableBlock);
    }

    unsigned int size_;
    char names_[NUM_GRAPHABLE_DATA][MAX_NAME_LENGTH];
    float data_[NUM_GRAPHABLE_DATA];

    void addData(const char *name, float data) {
        for (unsigned int i = 0; i < size_; i++) {
            if (strncmp(name,names_[i],MAX_NAME_LENGTH-1) == 0) {
                data_[i] = data;
                return;
            }
        }
        assert(size_ + 1 < NUM_GRAPHABLE_DATA);
        strncpy(names_[size_],name,MAX_NAME_LENGTH-1);
        names_[size_][MAX_NAME_LENGTH-1] = '\0';
        data_[size_] = data;
        size_++;
    }

    float getData(const char *name) {
        for (unsigned int i = 0; i < size_; i++) {
            if (strncmp(name,names_[i],MAX_NAME_LENGTH) == 0) {
                return data_[i];
            }
        }
        return -99999;
    }
};

#endif /* end of include guard: GRAPHABLEBLOCK_8VXFI3H7 */
