#ifndef MOTION_CORE_2H7QHCY7
#define MOTION_CORE_2H7QHCY7

#include <memory/Memory.h>
#include <memory/FrameInfoBlock.h>
#include <memory/WalkRequestBlock.h> // TEMPORARY TODO
#include <memory/Logger.h>

class KinematicsModule;
class MotionModule;
class SensorModule;

enum CoreType {
    CORE_ROBOT,
    CORE_SIM,
    CORE_TOOL
};

class MotionCore {
public:
    // DANIEL ADDED LAST PARAM
    MotionCore(CoreType type, bool use_shared_memory, Memory& memory);
    ~MotionCore();

    void move(float xvel, float yvel, float rotvel);
    void move(WalkRequestBlock::ParamSet paramSet, float xvel, float yvel, float rotvel);
    void processMotionFrame();
    void processSensorUpdate();

    void logMemory();

    Memory memory_;
    CoreType type_;
    unsigned int last_frame_processed_;

    KinematicsModule *kinematics_;
    MotionModule *motion_;
    SensorModule *sensor_;

#ifndef SWIG   // Lua can't handle the file IO
    Logger log_; //handling logging in vision for now
#endif

    static MotionCore *inst_;

    FrameInfoBlock *frame_info_;
    WalkRequestBlock *walk_request_; // TEMPORARY TODO
    double start_time_; // temporary
    double fps_time_;
    unsigned int fps_frames_processed_;

private:
    void initMemory();
    void initModules();
};

#endif /* end of include guard: CORE_2H7QHCY7 */
