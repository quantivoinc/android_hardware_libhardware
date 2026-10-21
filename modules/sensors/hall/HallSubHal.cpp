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

#include "HallContext.h"

#include <V2_1/SubHal.h>
#include <convertV2_1.h>

#include <mutex>
#include <vector>

#include <unistd.h>

using ::android::sp;
using ::android::hardware::hidl_handle;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::RateLevel;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SharedMemInfo;
using ::android::hardware::sensors::V2_0::implementation::ScopedWakelock;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;
using ::android::hardware::sensors::V2_1::implementation::IHalProxyCallback;
using ::android::hardware::sensors::V2_1::implementation::ISensorsSubHal;
using ::android::hardware::sensors::V2_1::implementation::convertFromSensorEvent;
using ::android::hardware::Void;

namespace {

Result ToResult(int rc) {
    if (rc == 0) {
        return Result::OK;
    }
    if (rc == -EINVAL) {
        return Result::BAD_VALUE;
    }
    return Result::INVALID_OPERATION;
}

class HallSubHal : public ISensorsSubHal {
  public:
    HallSubHal() : mHall(nullptr) {}

    const std::string getName() override { return "hall"; }

    Return<Result> initialize(const sp<IHalProxyCallback>& halProxyCallback) override {
        {
            std::lock_guard<std::mutex> guard(mCallbackLock);
            mCallback = halProxyCallback;
        }
        mHall.activate(HallContext::kHallHandle, 0);
        mHall.setEventListener([this](const sensors_event_t& event) { postEvent(event); });
        return Result::OK;
    }

    Return<void> getSensorsList_2_1(getSensorsList_2_1_cb callback) override {
        const sensor_t* list = nullptr;
        const int count = HallContext::GetSensorsList(&list);
        hidl_vec<SensorInfo> sensors;
        sensors.resize(count);
        for (int i = 0; i < count; i++) {
            const sensor_t& src = list[i];
            SensorInfo& info = sensors[i];
            info.sensorHandle = src.handle;
            info.name = src.name;
            info.vendor = src.vendor;
            info.version = src.version;
            info.type = static_cast<SensorType>(src.type);
            info.typeAsString = src.stringType != nullptr ? src.stringType : "";
            info.maxRange = src.maxRange;
            info.resolution = src.resolution;
            info.power = src.power;
            info.minDelay = src.minDelay;
            info.fifoReservedEventCount = src.fifoReservedEventCount;
            info.fifoMaxEventCount = src.fifoMaxEventCount;
            info.requiredPermission =
                    src.requiredPermission != nullptr ? src.requiredPermission : "";
            info.maxDelay = src.maxDelay;
            info.flags = src.flags;
        }
        callback(sensors);
        return Void();
    }

    Return<Result> setOperationMode(OperationMode mode) override {
        return mode == OperationMode::NORMAL ? Result::OK : Result::BAD_VALUE;
    }

    Return<Result> activate(int32_t sensorHandle, bool enabled) override {
        return ToResult(mHall.activate(sensorHandle, enabled ? 1 : 0));
    }

    Return<Result> batch(int32_t sensorHandle, int64_t samplingPeriodNs,
                         int64_t maxReportLatencyNs) override {
        return ToResult(mHall.batch(sensorHandle, 0, samplingPeriodNs, maxReportLatencyNs));
    }

    Return<Result> flush(int32_t sensorHandle) override {
        return ToResult(mHall.flush(sensorHandle));
    }

    Return<Result> injectSensorData_2_1(const Event& /*event*/) override {
        return Result::INVALID_OPERATION;
    }

    Return<void> registerDirectChannel(const SharedMemInfo& /*mem*/,
                                       registerDirectChannel_cb callback) override {
        callback(Result::INVALID_OPERATION, -1);
        return Void();
    }

    Return<Result> unregisterDirectChannel(int32_t /*channelHandle*/) override {
        return Result::INVALID_OPERATION;
    }

    Return<void> configDirectReport(int32_t /*sensorHandle*/, int32_t /*channelHandle*/,
                                    RateLevel /*rate*/, configDirectReport_cb callback) override {
        callback(Result::INVALID_OPERATION, 0);
        return Void();
    }

    Return<void> debug(const hidl_handle& fd, const hidl_vec<hidl_string>& /*args*/) override {
        if (fd.getNativeHandle() == nullptr || fd->numFds < 1) {
            return Void();
        }
        dprintf(fd->data[0], "Hall subhal\n");
        return Void();
    }

  private:
    void postEvent(const sensors_event_t& src) {
        sp<IHalProxyCallback> callback;
        {
            std::lock_guard<std::mutex> guard(mCallbackLock);
            callback = mCallback;
        }
        if (callback == nullptr) {
            return;
        }

        Event event;
        convertFromSensorEvent(src, &event);
        std::vector<Event> events;
        events.push_back(event);
        ScopedWakelock wakelock = callback->createScopedWakelock(false);
        callback->postEvents(events, std::move(wakelock));
    }

    HallContext mHall;
    std::mutex mCallbackLock;
    sp<IHalProxyCallback> mCallback;
};

}  // namespace

extern "C" __attribute__((visibility("default"))) ISensorsSubHal* sensorsHalGetSubHal_2_1(
        uint32_t* version) {
    static HallSubHal subHal;
    *version = SUB_HAL_2_1_VERSION;
    return &subHal;
}
