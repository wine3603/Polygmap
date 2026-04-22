#ifndef BRAINCO_HAND_CONTROLLER_H_
#define BRAINCO_HAND_CONTROLLER_H_
#include <stdint.h>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <condition_variable>
#include <array>

#include "hand_sdk.h"
#include "gesture_types.h"
#include "dexhand_def.h"
namespace eef_controller {

class BrainCoController;
using BrainCoControllerPtr = std::unique_ptr<BrainCoController>;
class BrainCoController {
public:
    struct HandStatus {
        std::array<int8_t, 6> position;
        std::array<int8_t, 6> speed;
        std::array<int8_t, 6> current;

        HandStatus(): position({0, 0, 0, 0, 0, 0}), speed({0, 0, 0, 0, 0, 0}), current({0, 0, 0, 0, 0, 0}) {}
    };
    static BrainCoControllerPtr Create(const std::string &gesture_filepath) {
        return std::unique_ptr<BrainCoController>(new BrainCoController(gesture_filepath));
    }

    ~BrainCoController();

    bool init();
    bool close();
    void send_position(HandSide hand_side, int8_t *pos);
    void send_speed(HandSide hand_side, int8_t *speed);
    void send_current(HandSide hand_side, int8_t *current);
    int getHandPort(int i);
    const std::array<HandStatus, 2>& get_status() const;

    /** 
     * @brief execute a gesture 
     * @param hand_side, the hand side to execute the gesture
     * @param gesture_name, the name of the gesture
     * @param [out] err_msg, the error message
     * @return true if success, false otherwise
     */
    bool execute_gesture(HandSide hand_side, const std::string &gesture_name, std::string &err_msg);
    
    /**
     * @brief  execute a list of gestures
     * 
     * @param gesture_tasks  the list of gestures to execute
     * @param err_msg        the error message
     * @return true if success, false otherwise 
     */
    bool execute_gestures(const GestureExecuteInfoVec& gesture_tasks, std::string &err_msg);

    /**
     * @brief get the list of gestures.
     * 
     * @return std::vector<GestureInfoMsg> 
     */
    std::vector<GestureInfoMsg> list_gestures();

    /**
     * @brief check if a gesture is currently executing.
     * 
     * @return true if a gesture is currently executing, false otherwise
     */
    bool is_gesture_executing();

private:
    BrainCoController(const std::string &gesture_filepath);
    void gesture_thread_func();
    bool sleep_for_100ms(int ms_count);

private:
    HandSDK *hand_sdk_ptr_ = nullptr;                               /* hand_sdk wrapper */
    std::string gesture_file_path_;                                 /* gesture config file */
    std::unordered_map<std::string, GestureInfoPtr> gesture_map_;   /* gesture info map */

    /* Task Queue Define! */
    struct GestureExecuteTask {
        HandSide        hand_side;  /* which hand */
        GestureInfoPtr  gesture;    /* gesture */
    };
    using BatchGestureTask = std::vector<GestureExecuteTask>;
    const int kTaskQueueSize_ = 1;            
    std::queue<BatchGestureTask> task_queue_;
    std::mutex task_queue_mtx_;
    std::condition_variable gesture_execute_cv_;
    std::atomic_bool abort_running_task_flag_ = false;  /* abort task execute! */
    std::atomic_bool gesture_executing_ = false;

    /* WARN FIXME: finger_status_ is the last sent position value, not read from sensor */
    std::array<HandStatus, 2> finger_status_;

    /* thread control */
    std::atomic_bool gesture_thread_exit_ = false;
    std::unique_ptr<std::thread> gesture_thread_ = nullptr;
};


} // namespace eef_controller

#endif