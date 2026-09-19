/* Shared-memory transport for simulator color-camera frames. */

#ifndef STONEFISH_ROS2_SIM_CAMERA_SHM_H
#define STONEFISH_ROS2_SIM_CAMERA_SHM_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace sf
{

/**
 * Writes RGB8 camera frames to a small latest-frame ring in POSIX shared
 * memory.  The shared-memory name and frame sequence are the IPC contract;
 * no process-local pointer is ever exposed to the reader.
 */
class SimCameraShmWriter
{
public:
    explicit SimCameraShmWriter(std::string channel, unsigned int slot_count = 2);
    ~SimCameraShmWriter();

    SimCameraShmWriter(const SimCameraShmWriter&) = delete;
    SimCameraShmWriter& operator=(const SimCameraShmWriter&) = delete;

    bool write(const void* data, std::size_t bytes, unsigned int width,
               unsigned int height, unsigned int step,
               std::int64_t stamp_sec, std::int64_t stamp_nsec);

private:
    bool open(unsigned int width, unsigned int height, unsigned int step);
    void close();

    std::string channel_;
    std::string name_;
    int fd_ = -1;
    void* mapping_ = nullptr;
    std::size_t mapping_size_ = 0;
    std::size_t slot_bytes_ = 0;
    unsigned int slot_count_ = 2;
};

}  // namespace sf

#endif
