#ifndef __SEEKER_HPP__
#define __SEEKER_HPP__

#include "seeker_type.hpp"

namespace SEEKNS {

// 数据读取接口（支持回调）
typedef std::function<void(event_header_t&, device_event_t&)> EventCallback;
typedef std::function<void(event_header_t&, const uint8_t*, int)> DataCallback;
typedef std::function<bool(std::pair<uint64_t, uint64_t>&, std::pair<uint64_t, uint64_t>&)> TimerCallback;

class SEEKER {
public:
    // 禁用拷贝构造函数和赋值操作符
    SEEKER(const SEEKER&) = delete;
    SEEKER& operator=(const SEEKER&) = delete;
    SEEKER();
    ~SEEKER();

    // 设备管理
    int init();
    int deinit();
    std::vector<seeker_device_t> find_devices();

    // 设备操作
    int open(seeker_device_t &sdev);
    int close();

    // 数据流控制
    int start_event_stream();
    int start_image_stream();
    int start_depth_stream();
    int stop_event_stream();
    int stop_image_stream();
    int stop_depth_stream();

    void set_event_callback(const EventCallback& callback);
    void set_mjpeg_callback(const DataCallback& callback);
    void set_depth_callback(const DataCallback& callback);
    void set_timer_callback(const TimerCallback& callback);

    int get_dev_info(seeker_device_t &sdev);

    // 发送命令
    int cmd_set_cali_cam(event_dev_cali_t& cali);
    int cmd_set_reboot();

    int cmd_get_cali_cam();
    int cmd_get_cali_depth();

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // SEEKNS

#endif // __SEEKER_HPP__
