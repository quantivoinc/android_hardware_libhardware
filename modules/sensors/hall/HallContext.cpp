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

#define LOG_TAG "HallSensor"

#include "HallContext.h"

#include <cutils/properties.h>
#include <log/log.h>
#include <linux/hidraw.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

constexpr float kDefaultRangeDeg = 359.0f;
constexpr int kDefaultVid = 0x2341;
constexpr int kDefaultPid = 0x0074;
constexpr int kReportIdDegrees = 0x01;
constexpr int kReportIdFan = 0x03;
constexpr int kReportIdConsumer = 0x04;
constexpr size_t kDegreeBytes = 2;
constexpr uint16_t kMaxDegrees = 359;

const sensor_t kHallSensor = {
        .name = "Hall Effect Sensor",
        .vendor = "The LineageOS Project",
        .version = 1,
        .handle = HallContext::kHallHandle,
        .type = SENSOR_TYPE_DEVICE_PRIVATE_BASE + 1,
        .maxRange = kDefaultRangeDeg,
        .resolution = 1.0f,
        .power = 0.5f,
        .minDelay = HallContext::kMinDelayUs,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = "org.lineageos.sensor.hall",
        .requiredPermission = "",
        .maxDelay = HallContext::kMaxDelayUs,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE,
        .reserved = {},
};

int LoadHexId(const char* prop, int fallback) {
    char fallbackBuf[8];
    snprintf(fallbackBuf, sizeof(fallbackBuf), "%04x", fallback);
    char buf[PROPERTY_VALUE_MAX];
    property_get(prop, buf, fallbackBuf);
    char* end = nullptr;
    const long value = strtol(buf, &end, 16);
    if (end == buf || value < 0 || value > 0xffff) {
        return fallback;
    }
    return static_cast<int>(value);
}

// Report id 1, one uint16 LE absolute field, logical range 0..359.
// hidraw prefixes the report id, so a full read is [0x01, lo, hi].
bool ParseDegrees(const uint8_t* buf, ssize_t length, uint16_t* degrees) {
    if (length < static_cast<ssize_t>(kDegreeBytes + 1) || buf[0] != kReportIdDegrees) {
        return false;
    }
    const uint16_t value =
            static_cast<uint16_t>(buf[1] | (static_cast<uint16_t>(buf[2]) << 8));
    if (value > kMaxDegrees) {
        return false;
    }
    *degrees = value;
    return true;
}

}  // namespace

HallContext::HallContext(const hw_module_t* module)
    : mVid(kDefaultVid),
      mPid(kDefaultPid),
      mHaveSample(false),
      mLastDegrees(0),
      mDevFd(-1),
      mStopFd(-1),
      mEnabled(false),
      mSamplingPeriodNs(kDefaultPeriodNs),
      mReader(),
      mReaderStarted(false) {
    memset(&device, 0, sizeof(device));
    device.common.tag = HARDWARE_DEVICE_TAG;
    device.common.version = SENSORS_DEVICE_API_VERSION_1_4;
    device.common.module = const_cast<hw_module_t*>(module);
    device.common.close = CloseWrapper;
    device.activate = ActivateWrapper;
    device.setDelay = SetDelayWrapper;
    device.poll = PollWrapper;
    device.batch = BatchWrapper;
    device.flush = FlushWrapper;
    device.inject_sensor_data = InjectWrapper;

    mVid = LoadHexId("persist.vendor.sensors.hall.vid", kDefaultVid);
    mPid = LoadHexId("persist.vendor.sensors.hall.pid", kDefaultPid);

    mStopFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (mStopFd < 0) {
        ALOGE("eventfd failed: %s", strerror(errno));
    }
    ALOGI("Hall degrees hid=%04x:%04x", mVid, mPid);
}

HallContext::~HallContext() {
    if (mEnabled.load()) {
        activate(kHallHandle, 0);
    }
    closeHid();
    if (mStopFd >= 0) {
        ::close(mStopFd);
        mStopFd = -1;
    }
}

int HallContext::GetSensorsList(sensor_t const** list) {
    *list = &kHallSensor;
    return 1;
}

int HallContext::close() {
    delete this;
    return 0;
}

int HallContext::activate(int handle, int enabled) {
    if (handle != kHallHandle) {
        return -EINVAL;
    }

    std::lock_guard<std::mutex> activateGuard(mActivateLock);

    if (!enabled) {
        mEnabled.store(false);
        if (mStopFd >= 0) {
            const uint64_t one = 1;
            if (write(mStopFd, &one, sizeof(one)) < 0 && errno != EAGAIN) {
                ALOGW("stop eventfd write failed: %s", strerror(errno));
            }
        }
        if (mReaderStarted) {
            pthread_join(mReader, nullptr);
            mReaderStarted = false;
        }
        closeHid();
        mHaveSample = false;
        mLastDegrees = 0;
        std::lock_guard<std::mutex> guard(mLock);
        mQueue.clear();
        return 0;
    }

    if (mEnabled.load()) {
        return 0;
    }

    if (openHid() != 0) {
        return -ENODEV;
    }

    if (mStopFd >= 0) {
        uint64_t drained = 0;
        while (read(mStopFd, &drained, sizeof(drained)) > 0) {
            // Drop stop tokens left from the previous deactivate.
        }
    }

    mEnabled.store(true);
    const int rc = pthread_create(&mReader, nullptr, ReaderTrampoline, this);
    if (rc != 0) {
        ALOGE("pthread_create failed: %s", strerror(rc));
        mEnabled.store(false);
        closeHid();
        return -rc;
    }
    mReaderStarted = true;
    return 0;
}

int HallContext::setDelay(int handle, int64_t samplingPeriodNs) {
    if (handle != kHallHandle) {
        return -EINVAL;
    }
    updatePeriodNs(samplingPeriodNs);
    return 0;
}

int HallContext::pollEvents(sensors_event_t* data, int count) {
    if (data == nullptr || count <= 0) {
        return -EINVAL;
    }

    std::unique_lock<std::mutex> guard(mLock);
    mCond.wait(guard, [this] { return !mQueue.empty(); });

    int written = 0;
    while (written < count && !mQueue.empty()) {
        data[written++] = mQueue.front();
        mQueue.pop_front();
    }
    return written;
}

int HallContext::batch(int handle, int flags, int64_t samplingPeriodNs,
                       int64_t /*maxReportLatencyNs*/) {
    if (handle != kHallHandle) {
        return -EINVAL;
    }
    if (flags & SENSORS_BATCH_DRY_RUN) {
        return 0;
    }
    updatePeriodNs(samplingPeriodNs);
    return 0;
}

int HallContext::flush(int handle) {
    if (handle != kHallHandle) {
        return -EINVAL;
    }
    if (!mEnabled.load()) {
        return -EINVAL;
    }
    dispatch(makeFlushEvent());
    return 0;
}

void HallContext::setEventListener(std::function<void(const sensors_event_t&)> listener) {
    std::lock_guard<std::mutex> guard(mLock);
    mListener = std::move(listener);
}

int HallContext::injectSensorData(const sensors_event_t* /*data*/) {
    return -EINVAL;
}

void HallContext::updatePeriodNs(int64_t samplingPeriodNs) {
    const int64_t minNs = static_cast<int64_t>(kMinDelayUs) * 1000LL;
    const int64_t maxNs = static_cast<int64_t>(kMaxDelayUs) * 1000LL;
    if (samplingPeriodNs < minNs) {
        samplingPeriodNs = minNs;
    } else if (samplingPeriodNs > maxNs) {
        samplingPeriodNs = maxNs;
    }
    mSamplingPeriodNs.store(samplingPeriodNs);
}

int HallContext::openHid() {
    if (mDevFd >= 0) {
        return 0;
    }
    for (int i = 0; i < 32; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/hidraw%d", i);
        const int fd = open(path, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
        if (fd < 0) {
            continue;
        }
        struct hidraw_devinfo info;
        memset(&info, 0, sizeof(info));
        if (ioctl(fd, HIDIOCGRAWINFO, &info) == 0 && info.vendor == mVid && info.product == mPid) {
            ALOGI("opened %s for %04x:%04x", path, mVid, mPid);
            mDevFd = fd;
            return 0;
        }
        ::close(fd);
    }
    ALOGE("USB HID %04x:%04x not found", mVid, mPid);
    return -ENODEV;
}

void HallContext::closeHid() {
    if (mDevFd >= 0) {
        ::close(mDevFd);
        mDevFd = -1;
    }
}

void* HallContext::ReaderTrampoline(void* arg) {
    static_cast<HallContext*>(arg)->readerLoop();
    return nullptr;
}

bool HallContext::waitForStop(int timeoutMs) {
    if (!mEnabled.load()) {
        return true;
    }
    if (mStopFd < 0) {
        usleep(static_cast<useconds_t>(timeoutMs) * 1000);
        return !mEnabled.load();
    }

    struct pollfd pfd;
    pfd.fd = mStopFd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    poll(&pfd, 1, timeoutMs);
    return !mEnabled.load() || (pfd.revents & POLLIN);
}

void HallContext::readerLoop() {
    while (mEnabled.load()) {
        if (mDevFd < 0) {
            if (openHid() != 0) {
                if (waitForStop(500)) {
                    return;
                }
                continue;
            }
        }

        if (!drainHidReports()) {
            ALOGW("hidraw lost, reopening");
            closeHid();
            continue;
        }
        if (mHaveSample) {
            publish(mLastDegrees);
        }

        const int64_t periodNs = mSamplingPeriodNs.load();
        int64_t leftNs = periodNs;
        while (mEnabled.load() && leftNs > 0) {
            int timeoutMs = static_cast<int>(leftNs / 1000000LL);
            if (timeoutMs < 1) {
                timeoutMs = 1;
            }

            struct pollfd fds[2];
            fds[0].fd = mDevFd;
            fds[0].events = POLLIN;
            fds[0].revents = 0;
            fds[1].fd = mStopFd;
            fds[1].events = POLLIN;
            fds[1].revents = 0;
            const nfds_t nfds = mStopFd >= 0 ? 2 : 1;

            const int64_t before = elapsedRealtimeNs();
            const int rc = poll(fds, nfds, timeoutMs);
            if (!mEnabled.load()) {
                return;
            }
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                ALOGE("poll failed: %s", strerror(errno));
                return;
            }
            if (mStopFd >= 0 && (fds[1].revents & POLLIN)) {
                return;
            }
            if ((fds[0].revents & (POLLNVAL | POLLHUP)) && !(fds[0].revents & POLLIN)) {
                ALOGW("hidraw hangup, reopening");
                closeHid();
                break;
            }
            if (fds[0].revents & POLLIN) {
                if (!drainHidReports()) {
                    ALOGW("hidraw lost, reopening");
                    closeHid();
                    break;
                }
            }

            const int64_t elapsed = elapsedRealtimeNs() - before;
            leftNs = elapsed >= leftNs ? 0 : leftNs - elapsed;
        }
    }
}

bool HallContext::drainHidReports() {
    uint8_t buf[64];
    while (mEnabled.load()) {
        const ssize_t n = read(mDevFd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return true;
            }
            ALOGE("hidraw read failed: %s", strerror(errno));
            return false;
        }
        if (n == 0) {
            return true;
        }
        if (n >= 1 && (buf[0] == kReportIdFan || buf[0] == kReportIdConsumer)) {
            continue;
        }
        uint16_t degrees = 0;
        if (!ParseDegrees(buf, n, &degrees)) {
            ALOGW("ignored HID report id %#x, %zd bytes", n > 0 ? buf[0] : 0, n);
            continue;
        }
        mLastDegrees = degrees;
        mHaveSample = true;
    }
    return true;
}

void HallContext::publish(uint16_t degrees) {
    ALOGV("hall degrees=%u", degrees);
    dispatch(makeSampleEvent(static_cast<float>(degrees)));
}

void HallContext::dispatch(const sensors_event_t& event) {
    std::function<void(const sensors_event_t&)> listener;
    {
        std::lock_guard<std::mutex> guard(mLock);
        enqueueLocked(event);
        listener = mListener;
        mCond.notify_one();
    }
    if (listener) {
        listener(event);
    }
}

void HallContext::enqueueLocked(const sensors_event_t& event) {
    while (mQueue.size() >= kMaxQueuedEvents) {
        mQueue.pop_front();
    }
    mQueue.push_back(event);
}

int64_t HallContext::elapsedRealtimeNs() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

sensors_event_t HallContext::makeSampleEvent(float degrees) {
    sensors_event_t event;
    memset(&event, 0, sizeof(event));
    event.version = sizeof(event);
    event.sensor = kHallHandle;
    event.type = SENSOR_TYPE_DEVICE_PRIVATE_BASE + 1;
    event.timestamp = elapsedRealtimeNs();
    event.data[0] = degrees;
    return event;
}

sensors_event_t HallContext::makeFlushEvent() {
    sensors_event_t event;
    memset(&event, 0, sizeof(event));
    event.version = sizeof(event);
    event.sensor = 0;
    event.type = SENSOR_TYPE_META_DATA;
    event.timestamp = elapsedRealtimeNs();
    event.meta_data.what = META_DATA_FLUSH_COMPLETE;
    event.meta_data.sensor = kHallHandle;
    return event;
}

int HallContext::CloseWrapper(hw_device_t* dev) {
    return reinterpret_cast<HallContext*>(dev)->close();
}

int HallContext::ActivateWrapper(sensors_poll_device_t* dev, int handle, int enabled) {
    return reinterpret_cast<HallContext*>(dev)->activate(handle, enabled);
}

int HallContext::SetDelayWrapper(sensors_poll_device_t* dev, int handle, int64_t samplingPeriodNs) {
    return reinterpret_cast<HallContext*>(dev)->setDelay(handle, samplingPeriodNs);
}

int HallContext::PollWrapper(sensors_poll_device_t* dev, sensors_event_t* data, int count) {
    return reinterpret_cast<HallContext*>(dev)->pollEvents(data, count);
}

int HallContext::BatchWrapper(sensors_poll_device_1* dev, int handle, int flags,
                              int64_t samplingPeriodNs, int64_t maxReportLatencyNs) {
    return reinterpret_cast<HallContext*>(dev)->batch(handle, flags, samplingPeriodNs,
                                                      maxReportLatencyNs);
}

int HallContext::FlushWrapper(sensors_poll_device_1* dev, int handle) {
    return reinterpret_cast<HallContext*>(dev)->flush(handle);
}

int HallContext::InjectWrapper(sensors_poll_device_1* dev, const sensors_event_t* data) {
    return reinterpret_cast<HallContext*>(dev)->injectSensorData(data);
}

static int open_sensors(const hw_module_t* module, const char* /*name*/, hw_device_t** dev) {
    HallContext* ctx = new HallContext(module);
    *dev = &ctx->device.common;
    return 0;
}

static hw_module_methods_t gHallModuleMethods = {
        .open = open_sensors,
};

static int get_sensors_list(sensors_module_t* /*module*/, sensor_t const** list) {
    return HallContext::GetSensorsList(list);
}

static int set_operation_mode(unsigned int mode) {
    return mode == SENSOR_HAL_NORMAL_MODE ? 0 : -EINVAL;
}

sensors_module_t HAL_MODULE_INFO_SYM = {
        .common =
                {
                        .tag = HARDWARE_MODULE_TAG,
                        .version_major = 1,
                        .version_minor = 0,
                        .id = SENSORS_HARDWARE_MODULE_ID,
                        .name = "Hall-effect sensor subhal",
                        .author = "The LineageOS Project",
                        .methods = &gHallModuleMethods,
                        .dso = nullptr,
                        .reserved = {0},
                },
        .get_sensors_list = get_sensors_list,
        .set_operation_mode = set_operation_mode,
};
