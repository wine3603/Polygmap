#ifndef _TOUCH_DEXHAND_H_
#define _TOUCH_DEXHAND_H_
#include <memory>
#include <mutex>
#include <array>
#include <iostream>
#include <vector>
struct ModbusHandle;
namespace dexhand {

enum DexHandType : uint8_t {
  SKU_TYPE_MEDIUM_RIGHT = 1,
  SKU_TYPE_MEDIUM_LEFT = 2,
  SKU_TYPE_SMALL_RIGHT = 3,
  SKU_TYPE_SMALL_LEFT = 4,
  SKU_TYPE_NONE = 110,  

};

enum GripForce : uint8_t {
  FORCE_LEVEL_SMALL = 1,
  FORCE_LEVEL_NORMAL = 2,
  FORCE_LEVEL_FULL = 3,
};

/// 内置手势1~6：张开、握拳、两只捏、三只捏、侧边捏、单指点
/// 自定义手势6个: 10~15
enum ActionSequenceId_t : uint8_t {
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_OPEN = 1,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_FIST = 2,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_TWO = 3,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_THREE = 4,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_SIDE = 5,
  ACTION_SEQUENCE_ID_DEFAULT_GESTURE_POINT = 6,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE1 = 10,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE2 = 11,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE3 = 12,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE4 = 13,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE5 = 14,
  ACTION_SEQUENCE_ID_CUSTOM_GESTURE6 = 15,
};

struct TouchSensorStatus_t {
  uint16_t normal_force1;
  uint16_t normal_force2;
  uint16_t normal_force3;
  uint16_t tangential_force1;
  uint16_t tangential_force2;
  uint16_t tangential_force3;
  uint16_t tangential_direction1;
  uint16_t tangential_direction2;
  uint16_t tangential_direction3;
  uint32_t self_proximity1;
  uint32_t self_proximity2;
  uint32_t mutual_proximity;
  uint16_t status;
};

struct TurboConfig_t {
  uint16_t interval;
  uint16_t duration;
};

using FingerArray = std::array<int16_t, 6>;
using UnsignedFingerArray = std::array<uint16_t, 6>;
using TouchSensorStatusArray = std::array<TouchSensorStatus_t, 5>;
using FingerTouchStatusPtr = std::shared_ptr<TouchSensorStatusArray>;

struct ActionSeqDataType {
    uint16_t duration_ms; // ms
    UnsignedFingerArray positions;
    UnsignedFingerArray speeds;
    UnsignedFingerArray forces;

    ActionSeqDataType(): duration_ms(500) {
        for (int i = 0; i < 6; i++) {
            positions[i] = 0;
            speeds[i] = 0;
            forces[i] = 0;
        }
    }
};
using ActionSeqDataTypeVec = std::vector<ActionSeqDataType>;

/* open finger positions */
const UnsignedFingerArray kOpenFingerPositions = {
    0, 0, 0, 0, 0, 0
};
/* close finger positions */
const UnsignedFingerArray kCloseFingerPositions = {
    50, 90, 90, 90, 90, 90
};

struct DeviceInfo {
    DexHandType sku_type;
    std::string serial_number;
    std::string firmware_version;

    friend std::ostream& operator<<(std::ostream& os, const DeviceInfo& info) {
        os << "Sku Type: " << static_cast<int>(info.sku_type) << "\nSerial Number: " << info.serial_number << "\nFirmware Version: " << info.firmware_version << "\n";
        return os;
    }
};

struct FingerStatus {
    UnsignedFingerArray positions;
    FingerArray speeds;
    FingerArray currents;
    UnsignedFingerArray states;

    friend std::ostream& operator<<(std::ostream& os, const FingerStatus& status) {
        os << "Finger Positions: ";
        for (const auto& pos : status.positions) {
            os << pos << " ";
        }
        os << "\nFinger Speeds: ";
        for (const auto& speed : status.speeds) {
            os << speed << " ";
        }
        os << "\nFinger Currents: ";
        for (const auto& current : status.currents) {
            os << current << " ";
        }
        os << "\nFinger States: ";
        for (const auto& state : status.states) {
            os << state << " ";
        }
        return os;
    }
};
using FingerStatusPtr = std::shared_ptr<FingerStatus>;

std::ostream& operator<<(std::ostream& os, const TouchSensorStatus_t& status);
std::ostream& operator<<(std::ostream& os, const FingerStatusPtr& status);

class TouchDexhand {
public:
    TouchDexhand(const TouchDexhand&) = delete;
    TouchDexhand& operator=(const TouchDexhand&) = delete;
    ~TouchDexhand();

    /**
     * @brief   Connect to the device
     * 
     * @param port 串口名称，例如："/dev/ttyUSB0"
     * @param slave_id 设备ID，默认为1，范围为1~255, 0 为广播地址
     * @param baudrate  波特率，115200, 57600, 19200, 460800
     * @return std::unique_ptr<TouchDexhand> 失败返回 nullptr
     */
    static std::unique_ptr<TouchDexhand> Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    );

    /**
     * @brief Get the Device Info object
     * 
     * @return DeviceInfo 
     */
    DeviceInfo getDeviceInfo();

    /**
     * @brief Set the Finger Positions.
     * 
     * @param positions range: 0~100, 0 for open, 100 for close.
     */
    void setFingerPositions(const UnsignedFingerArray &positions);
    
    /**
     * @brief Set the Finger Speeds object
     * 
     * @param speeds range: -100~100
     * @note The fingers will move at the set speed values until they stall. 
     *       The value range is -100 to 100. Positive values indicate flexion, negative values indicate extension.
     */
    void setFingerSpeeds(const FingerArray &speeds);

    /**
     * @brief Get the Finger Status object
     * 
     * @return FingerStatusPtr 
     */
    FingerStatusPtr getFingerStatus();

    /**
     * @brief Set the Force Level object
     * 
     * @param level {GripForce::NORMAL, GripForce::SMALL, GripForce::FULL}
     */
    void setGripForce(GripForce level);

    /**
     * @brief Get the Force Level object
     * 
     * @return GripForce 
     */
    GripForce  getGripForce();

    /**
     * @brief Get the Touch Status object
     * 
     * @return FingerTouchStatusPtr 
     */
    FingerTouchStatusPtr getTouchStatus();

    /**
     * @brief Set the Turbo Mode Enabled object
     * @note 开启之后会持续握紧, 掉电后，Turbo 模式会恢复到默认关闭状态。
     * @param enabled 
     */
    void setTurboModeEnabled(bool enabled);

    /**
     * @brief Check if Turbo Mode is enabled
     * 
     * @return true if Turbo Mode is enabled
     * @return false if Turbo Mode is not enabled
     */
    bool isTurboModeEnabled();

    TurboConfig_t getTurboConfig();
    /**
     * @brief 重置触觉传感器采集通道
     * @note 在执行该指令时，手指传感器尽量不要受力, 0b00000001 表示重置第一个传感器
     * 
     * @param bits 
     */
    void resetTouchSensor(uint8_t bits = 0xFF);

    /**
     * @brief 启用触觉传感器
     * @note 0b00000001 表示启用第一个传感器
     * 
     * @param bits 
     */
    void enableTouchSensor(uint8_t bits = 0xFF);

    /**
     * @brief 运行动作序列
     * 
     * @param seq_id 
     */
    void runActionSequence(ActionSequenceId_t seq_id);

    /**
     * @brief 设置动作序列
     * 
     * @param seq_id 
     * @param sequences
     * @return true if success
     * @return false if failed
     */
    bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences);
    
private:
    explicit TouchDexhand(ModbusHandle* handle, uint8_t slave_id_);

private:
    ModbusHandle* mb_handle_ = nullptr;
    uint8_t slave_id_;
};
} // namespace dexhand
#endif