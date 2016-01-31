#include "MotionCore.h"
#include <memory/FrameInfoBlock.h>
#include <memory/GraphableBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/WalkEngineBlock.h>
#include <memory/WalkRequestBlock.h>

#include <kinematics/KinematicsModule.h>
#include <motion/MotionModule.h>
#include <sensor/SensorModule.h>

MotionCore *MotionCore::inst_ = NULL;

MotionCore::MotionCore (CoreType type, bool use_shared_memory, Memory& memory):
// DANIEL CHANGED THIS
//  memory_(use_shared_memory,false),
// TO
    memory_(memory),
    type_(type),
    last_frame_processed_(0),
    log_("/dev/null"),
    fps_frames_processed_(0)
{
    inst_ = this;
    initMemory();
    initModules();

    start_time_ = frame_info_->seconds_since_start;
    fps_time_ = frame_info_->seconds_since_start;
}

MotionCore::~MotionCore() {
    if (kinematics_ != NULL)
        delete kinematics_;
    if (motion_ != NULL)
        delete motion_;
    if (sensor_ != NULL)
        delete sensor_;
}

void MotionCore::move(float xvel, float yvel, float rotvel) {
    move(WalkRequestBlock::PARAMS_DEFAULT, xvel, yvel, rotvel);
}

void MotionCore::move(WalkRequestBlock::ParamSet paramSet, float xvel, float yvel, float rotvel) {
    static float timeLastMove = -100.0;
    const float COOL_DOWN_TIME = .5;

    if ((xvel == 0) && (yvel == 0) && (rotvel == 0) &&
            (frame_info_->seconds_since_start - timeLastMove >= COOL_DOWN_TIME)) {
        walk_request_->motion_ = WalkRequestBlock::STAND;
    } else {
        if ((xvel != 0) || (yvel != 0) || (rotvel != 0)) {
            timeLastMove = frame_info_->seconds_since_start;
        } else {
            //cout << "COOL DOWN" << endl;
        }
        walk_request_->motion_ = WalkRequestBlock::WALK;
        walk_request_->paramSet_ = paramSet;
        walk_request_->speed_.rotation = rotvel;
        walk_request_->speed_.translation = Vector2<float>(xvel,yvel);
        walk_request_->pedantic_walk_ = false;
        walk_request_->percentage_speed_ = true;
    }

    processMotionFrame();
}

void MotionCore::processMotionFrame() {
    unsigned int &frame_id = frame_info_->frame_id;
    if (frame_id <= last_frame_processed_) {
        std::cout << "processMotionFrame, skipping frame " << frame_id << std::endl;
        return;
    }
    // frame rate
    double time_passed = frame_info_->seconds_since_start - fps_time_;
    fps_frames_processed_++;
    if (fps_frames_processed_ == 100) {
        //std::cout << "MOTION FRAME RATE: " << fps_frames_processed_ / time_passed << std::endl;
        fps_frames_processed_ = 0;
        fps_time_ = frame_info_->seconds_since_start;
    }

    time_passed = frame_info_->seconds_since_start - start_time_;
    // for now, fill in the walk request // TEMPORARY TODO
    //walk_request_->motion_ = WalkRequestBlock::NONE;
    /*  if (time_passed > 12.0)
        walk_request_->motion_ = WalkRequestBlock::STAND;
      else if (time_passed > 10.0) {
        walk_request_->motion_ = WalkRequestBlock::WALK;
        walk_request_->speed_.rotation = 0.0;
        walk_request_->speed_.translation = Vector2<float>(0.0,0);
        walk_request_->pedantic_walk_ = false;
        walk_request_->percentage_speed_ = true;
      } else if (time_passed > 2.0) {
        walk_request_->motion_ = WalkRequestBlock::WALK;
        walk_request_->speed_.rotation = 0.0;
        walk_request_->speed_.translation = Vector2<float>(1.0,0);
        walk_request_->pedantic_walk_ = false;
        walk_request_->percentage_speed_ = true;
      } else {
        walk_request_->motion_ = WalkRequestBlock::STAND;
      }
    */

    // actually do stuff
    processSensorUpdate();
    motion_->processFrame();

    //logMemory();
    //kinematics_->calculatePose();
    //std::cerr << ((BodyModelBlock*)memory_.getBlockPtr("body_model"))->center_of_mass_ << std::endl;

    last_frame_processed_ = frame_id;
}

void MotionCore::processSensorUpdate() {
    sensor_->processSensors();
    kinematics_->calculatePose();
}

void MotionCore::initModules() {
    kinematics_ = new KinematicsModule();
    kinematics_->init(&(memory_));

    motion_ = new MotionModule();
    motion_->init(&(memory_));
    sensor_ = new SensorModule();
    sensor_->init(&(memory_));
}

void MotionCore::initMemory() {
    // create all the modules that are used
    MemorySource mem_source = MEMORY_SIM;
    if (type_ == CORE_ROBOT)
        mem_source = MEMORY_ROBOT;

    //Add required memory blocks
    memory_.addBlockByName("processed_sensors");
    memory_.addBlockByName("body_model");
    memory_.addBlockByName("graphable");
    memory_.addBlockByName("walk_engine");
    memory_.addBlockByName("walk_request");
    memory_.addBlockByName("odometry");

    memory_.getBlockByName(frame_info_,"frame_info");
    memory_.getBlockByName(walk_request_,"walk_request");

    // print out all the memory blocks we're using
    /*
    std::vector<std::string> memory_block_names;
    memory_.getBlockNames(memory_block_names,false);
    std::cout << "INITIAL MEMORY BLOCKS:" << std::endl;
    for (unsigned int i = 0; i < memory_block_names.size(); i++)
      std::cout << memory_block_names[i] << std::endl;
    std::cout << "--------------" << std::endl;
    */
}

void MotionCore::logMemory() {
    log_.writeMemory(memory_);
}
