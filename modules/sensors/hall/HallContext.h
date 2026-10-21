/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HARDWARE_LIBHARDWARE_MODULES_SENSORS_HALL_HALLCONTEXT_H_
#define HARDWARE_LIBHARDWARE_MODULES_SENSORS_HALL_HALLCONTEXT_H_

#include <hardware/sensors.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>

#include <pthread.h>

/*
 * Sensors sub-HAL for a USB HID Hall angle sensor.
 *
 * sensors-multihal dlopens this module from /vendor/etc/sensors/hals.conf and
 * calls sensorsHalGetSubHal_2_1(). The sensor is continuous:
 *
 *   data[0] = angle in degrees, 0..359
 *
 * Source is USB HID 2341:0074. The device has three reports:
 *   1  input   uint16 degrees, 0..359. This sub-HAL reads it from hidraw:
 *              [0x01][degrees_lo][degrees_hi]
 *   3  output  uint8 fan duty, 0..255. usbhid_fan sends it.
 *   4  input   uint16 consumer usage. The kernel maps AC Zoom In/Out to keys.
 * Reports 3 and 4 are not sensor samples and are skipped here.
 *
 * persist.vendor.sensors.hall.vid  hex vendor id (default 2341)
 * persist.vendor.sensors.hall.pid  hex product id (default 0074)
 */

class HallContext {
  public:
    explicit HallContext(const hw_module_t* module);
    ~HallContext();

    sensors_poll_device_1_t device;

    int close();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t samplingPeriodNs);
    int pollEvents(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t samplingPeriodNs, int64_t maxReportLatencyNs);
    int flush(int handle);
    int injectSensorData(const sensors_event_t* data);

    static constexpr int kHallHandle = 1;
    static constexpr int32_t kMinDelayUs = 10000;
    static constexpr int32_t kMaxDelayUs = 1000000;
    static constexpr int64_t kDefaultPeriodNs = 200000000LL;

    static int GetSensorsList(sensor_t const** list);

    // Invoked without the queue lock. Used by the 2.1 sub-HAL to postEvents().
    void setEventListener(std::function<void(const sensors_event_t&)> listener);

  private:
    static constexpr size_t kMaxQueuedEvents = 64;

    void closeHid();
    void readerLoop();
    int openHid();
    bool drainHidReports();
    bool waitForStop(int timeoutMs);
    void publish(uint16_t degrees);
    void dispatch(const sensors_event_t& event);
    void enqueueLocked(const sensors_event_t& event);
    void updatePeriodNs(int64_t samplingPeriodNs);
    static int64_t elapsedRealtimeNs();
    static sensors_event_t makeSampleEvent(float degrees);
    static sensors_event_t makeFlushEvent();

    int mVid;
    int mPid;
    bool mHaveSample;
    uint16_t mLastDegrees;

    int mDevFd;
    int mStopFd;

    std::atomic<bool> mEnabled;
    std::atomic<int64_t> mSamplingPeriodNs;
    pthread_t mReader;
    bool mReaderStarted;

    std::mutex mActivateLock;
    std::mutex mLock;
    std::condition_variable mCond;
    std::deque<sensors_event_t> mQueue;
    std::function<void(const sensors_event_t&)> mListener;

    static void* ReaderTrampoline(void* arg);
    static int CloseWrapper(hw_device_t* dev);
    static int ActivateWrapper(sensors_poll_device_t* dev, int handle, int enabled);
    static int SetDelayWrapper(sensors_poll_device_t* dev, int handle, int64_t samplingPeriodNs);
    static int PollWrapper(sensors_poll_device_t* dev, sensors_event_t* data, int count);
    static int BatchWrapper(sensors_poll_device_1* dev, int handle, int flags,
                            int64_t samplingPeriodNs, int64_t maxReportLatencyNs);
    static int FlushWrapper(sensors_poll_device_1* dev, int handle);
    static int InjectWrapper(sensors_poll_device_1* dev, const sensors_event_t* data);
};

#endif  // HARDWARE_LIBHARDWARE_MODULES_SENSORS_HALL_HALLCONTEXT_H_
