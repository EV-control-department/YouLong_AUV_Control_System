/* Shared-memory transport for simulator color-camera frames. */

#include "stonefish_ros2/SimCameraShm.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <utility>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace sf
{
namespace
{

constexpr std::uint32_t kMagic = 0x55564331U;  // "UVC1"
constexpr std::uint32_t kVersion = 1U;
constexpr std::size_t kHeaderBytes = 64U;

// Keep this layout stable for the Python reader.  The sequence is a seqlock:
// odd while a slot is being written, even after the payload is complete.
struct alignas(64) ShmHeader
{
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t step;
    std::uint32_t channels;
    std::uint32_t slot_count;
    std::uint32_t reserved;
    std::uint64_t slot_bytes;
    std::uint64_t sequence;
    std::int64_t stamp_sec;
    std::int64_t stamp_nsec;
};

static_assert(sizeof(ShmHeader) == kHeaderBytes,
              "shared-memory header must remain 64 bytes");

std::string shared_name(const std::string& channel)
{
    std::string prefix = "/uv_sim_camera";
    if (const char* value = std::getenv("UV_SIM_SHM_PREFIX")) {
        if (*value != '\0') {
            prefix = value;
        }
    }
    if (prefix.front() != '/') {
        prefix.insert(prefix.begin(), '/');
    }
    while (prefix.size() > 1U && prefix.back() == '/') {
        prefix.pop_back();
    }
    std::string safe;
    safe.reserve(channel.size());
    for (char c : channel) {
        safe.push_back((c == '/' || c == ' ') ? '_' : c);
    }
    return prefix + "_" + safe;
}

}  // namespace

SimCameraShmWriter::SimCameraShmWriter(std::string channel,
                                       unsigned int slot_count)
    : channel_(std::move(channel)),
      name_(shared_name(channel_)),
      slot_count_(std::max(2U, slot_count))
{
}

SimCameraShmWriter::~SimCameraShmWriter()
{
    close();
}

bool SimCameraShmWriter::open(unsigned int width, unsigned int height,
                              unsigned int step)
{
    if (width == 0U || height == 0U || step < width * 3U) {
        return false;
    }
    slot_bytes_ = static_cast<std::size_t>(step) * height;
    mapping_size_ = kHeaderBytes + slot_bytes_ * slot_count_;

    fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd_ < 0) {
        return false;
    }
    if (ftruncate(fd_, static_cast<off_t>(mapping_size_)) != 0) {
        close();
        return false;
    }
    mapping_ = mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE,
                    MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
        mapping_ = nullptr;
        close();
        return false;
    }

    auto* header = static_cast<ShmHeader*>(mapping_);
    std::memset(mapping_, 0, mapping_size_);
    header->magic = kMagic;
    header->version = kVersion;
    header->width = width;
    header->height = height;
    header->step = step;
    header->channels = 3U;
    header->slot_count = slot_count_;
    header->slot_bytes = slot_bytes_;
    // Publish initialized metadata last enough that a reader never accepts a
    // zeroed mapping as a valid frame.
    __atomic_store_n(&header->sequence, static_cast<std::uint64_t>(0),
                     __ATOMIC_RELEASE);
    return true;
}

bool SimCameraShmWriter::write(const void* data, std::size_t bytes,
                               unsigned int width, unsigned int height,
                               unsigned int step, std::int64_t stamp_sec,
                               std::int64_t stamp_nsec)
{
    if (data == nullptr || bytes == 0U) {
        return false;
    }
    if (mapping_ == nullptr) {
        if (!open(width, height, step)) {
            return false;
        }
    }
    auto* header = static_cast<ShmHeader*>(mapping_);
    if (header->width != width || header->height != height
        || header->step != step || bytes > slot_bytes_) {
        close();
        if (!open(width, height, step)) {
            return false;
        }
        header = static_cast<ShmHeader*>(mapping_);
    }

    const std::uint64_t previous =
        __atomic_load_n(&header->sequence, __ATOMIC_ACQUIRE);
    const std::uint64_t frame_number = previous / 2U + 1U;
    const unsigned int slot = static_cast<unsigned int>(
        (frame_number - 1U) % static_cast<std::uint64_t>(slot_count_));
    __atomic_store_n(&header->sequence, frame_number * 2U + 1U,
                     __ATOMIC_RELEASE);
    auto* payload = static_cast<std::uint8_t*>(mapping_) + kHeaderBytes
        + static_cast<std::size_t>(slot) * slot_bytes_;
    std::memcpy(payload, data, bytes);
    if (bytes < slot_bytes_) {
        std::memset(payload + bytes, 0, slot_bytes_ - bytes);
    }
    header->stamp_sec = stamp_sec;
    header->stamp_nsec = stamp_nsec;
    __atomic_store_n(&header->sequence, frame_number * 2U,
                     __ATOMIC_RELEASE);
    return true;
}

void SimCameraShmWriter::close()
{
    if (mapping_ != nullptr) {
        munmap(mapping_, mapping_size_);
        mapping_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (!name_.empty()) {
        shm_unlink(name_.c_str());
    }
}

}  // namespace sf
