#pragma once

// 宿主(Host) LockedField — 单线程无锁版。
// 与固件 LockedField.hpp 同接口(get/set),但临界区退化为空(见 stubs/FreeRTOS.h),
// 因此 get/set 是纯内存拷贝。宿主桥(ControllerHost)串行调用,无需互斥。

#include <utility>

template <typename T>
class LockedField {
public:
  LockedField() = default;
  LockedField(const T &initial) : val_(initial) {}

  T get() const { return val_; }
  void set(const T &v) { val_ = v; }

  T &unsafe() { return val_; }
  const T &unsafe() const { return val_; }

private:
  T val_{};
};
