/// @file Driver wrapping the spinnaker SDK.
#include "device/props/camera.h"
#include "device/kit/camera.h"
#include "device/kit/driver.h"
#include "platform.h"
#include "logger.h"

#include "CameraDefs.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <stdexcept>
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <cstring>

#define LOG(...) aq_logger(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define LOGE(...) aq_logger(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            LOGE(__VA_ARGS__);                                                 \
            throw std::runtime_error("Expression was false: " #e);             \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false:\n\t%s", #e)

namespace {

//
// Utilities
//

template <typename T>
T
inv_at_or(
    const std::unordered_map<T, Spinnaker::GenICam::gcstring> & table,
    const Spinnaker::GenICam::gcstring & value,
    const T default_result)
{
    for (const auto & [k, v] : table) {
        if (v == value) {
            return k;
        }
    }
    return default_result;
}

const std::unordered_map<TriggerEdge, Spinnaker::GenICam::gcstring> trigger_edge_to_activation{
    { TriggerEdge_Rising, "RisingEdge" },
    { TriggerEdge_Falling, "FallingEdge" },
    { TriggerEdge_AnyEdge, "AnyEdge" },
    { TriggerEdge_LevelHigh, "LevelHigh" },
    { TriggerEdge_LevelLow, "LevelLow" },
};

TriggerEdge
to_trigger_edge(const Spinnaker::GenICam::gcstring & activation)
{
    return inv_at_or(trigger_edge_to_activation, activation, TriggerEdge_Unknown);
}

const Spinnaker::GenICam::gcstring &
to_trigger_activation(TriggerEdge edge)
{
    return trigger_edge_to_activation.at(edge);
}

const std::unordered_map<SampleType, Spinnaker::GenICam::gcstring> sample_type_to_pixel_format{
    { SampleType_u8, "Mono8" },
    { SampleType_i8, "Mono8s" },
    { SampleType_u10, "Mono10" },
    { SampleType_u12, "Mono12" },
    { SampleType_u14, "Mono14" },
    { SampleType_u16, "Mono16" },
    { SampleType_i16, "Mono16s" },
    { SampleType_f32, "Mono32f" },
};

SampleType
to_sample_type(const Spinnaker::GenICam::gcstring & pixel_format)
{
    return inv_at_or(sample_type_to_pixel_format, pixel_format, SampleType_Unknown);
}

const Spinnaker::GenICam::gcstring &
to_pixel_format(SampleType sample_type)
{
    return sample_type_to_pixel_format.at(sample_type);
}

// Define the software line as acquire's line 2 because the Blackfly USB3 camera
// only has two physical lines (0 and 1).
// Similarly define acquire's line 3 as unknown.

const std::unordered_map<uint8_t, Spinnaker::GenICam::gcstring> trigger_line_to_source{
    {0, "Line0"},
    {1, "Line1"},
    {2, "Software"},
};

uint8_t
to_trigger_line(const Spinnaker::GenICam::gcstring & source)
{
    return inv_at_or<uint8_t>(trigger_line_to_source, source, 3);
}

const Spinnaker::GenICam::gcstring &
to_trigger_source(uint8_t line)
{
    return trigger_line_to_source.at(line);
}

bool
is_equal(const Trigger& lhs, const Trigger& rhs)
{
    return memcmp(&lhs, &rhs, sizeof(Trigger)) == 0;
}

template<typename T>
static T
clamp(T val, float low, float high)
{
    float fval = float(val);
    return (fval < low)    ? static_cast<T>(low)
           : (fval > high) ? static_cast<T>(high)
                           : val;
}

void
check_spinnaker_camera_id(uint64_t id)
{
    constexpr uint64_t limit = std::numeric_limits<unsigned int>::max();
    EXPECT(id < limit, "Expected an unsigned int device index. Got: %llu", id);
}

//
// Camera declaration
//

struct SpinnakerCamera final : private Camera
{
    explicit SpinnakerCamera(Spinnaker::CameraPtr camera);
    ~SpinnakerCamera();

    void set(struct CameraProperties* properties);
    void get(struct CameraProperties* properties);
    void get_meta(struct CameraPropertyMetadata* meta) const;
    void get_shape(struct ImageShape* shape) const;
    void start();
    void stop();
    void execute_trigger() const;
    void get_frame(void* im, size_t* nbytes, struct ImageInfo* info);

  private:
    // Guards access to camera_.
    mutable std::mutex lock_;
    // Hold a reference to the camera as a concise way to get and set
    // its state (rather than through its GenICam node map).
    Spinnaker::CameraPtr camera_;
    uint64_t frame_id_;
    struct CameraProperties last_known_settings_;
    struct CameraPropertyMetadata last_known_capabilities_;

    void query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const;
    void query_binning_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_offset_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_shape_capabilities_(CameraPropertyMetadata* meta) const;
    void query_pixel_type_capabilities_(CameraPropertyMetadata* meta) const;
    static void query_triggering_capabilities_(CameraPropertyMetadata* meta);

    float maybe_set_exposure_time_us_(float target_us, float last_value_us);
    uint8_t maybe_set_binning(uint8_t target, uint8_t last_value);
    SampleType maybe_set_sample_type(SampleType target, SampleType last_known);
    CameraProperties::camera_properties_offset_s maybe_set_offset(
      CameraProperties::camera_properties_offset_s target,
      CameraProperties::camera_properties_offset_s last);
    CameraProperties::camera_properties_shape_s maybe_set_shape(
      CameraProperties::camera_properties_shape_s target,
      CameraProperties::camera_properties_shape_s last);
    Trigger& maybe_set_input_trigger_frame_start(Trigger& target,
                                                 const Trigger& last);
    Trigger& maybe_set_output_trigger_exposure(Trigger& target,
                                               const Trigger& last);
};

//
// Non-throwing camera implementation.
//

enum DeviceStatusCode
spinnakercam_set(struct Camera* self_, struct CameraProperties* settings)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->set(settings);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_get(const struct Camera* self_, struct CameraProperties* settings)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->get(settings);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_get_meta(const struct Camera* self_,
                      struct CameraPropertyMetadata* meta)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->get_meta(meta);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_get_shape(const struct Camera* self_, struct ImageShape* shape)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->get_shape(shape);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_stop(struct Camera* self_)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->stop();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_start(struct Camera* self_)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->start();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_execute_trigger(struct Camera* self_)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->execute_trigger();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_get_frame(struct Camera* self_,
                       void* im,
                       size_t* nbytes,
                       struct ImageInfo* info)
{
    try {
        CHECK(self_);
        ((struct SpinnakerCamera*)self_)->get_frame(im, nbytes, info);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

//
// Camera implementation
//

SpinnakerCamera::SpinnakerCamera(Spinnaker::CameraPtr camera)
  : Camera{ .set = ::spinnakercam_set,
            .get = ::spinnakercam_get,
            .get_meta = ::spinnakercam_get_meta,
            .get_shape = ::spinnakercam_get_shape,
            .start = ::spinnakercam_start,
            .stop = ::spinnakercam_stop,
            .execute_trigger = ::spinnakercam_execute_trigger,
            .get_frame = ::spinnakercam_get_frame,
  }
  , camera_(camera)
  , last_known_settings_{}
  , frame_id_(0)
{
    // Sometimes the camera is still initialized even after running CameraBase::DeInit
    // Ideally, we would error here, but that can make the camera indefinitely unusable
    // in Acquire, so log and try to recover instead.
    if (camera->IsInitialized()) {
        LOGE("Camera was already initialized. Stopping and de-initializing it before initialization.");
        stop();
        camera_->DeInit();
    }
    camera->Init();
    get(&last_known_settings_);
    get_meta(&last_known_capabilities_);
}

SpinnakerCamera::~SpinnakerCamera()
{
    try {
        stop();
        const std::scoped_lock lock(lock_);
        camera_->DeInit();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
}

void
SpinnakerCamera::set(struct CameraProperties* properties)
{
    const std::scoped_lock lock(lock_);

    last_known_settings_.exposure_time_us = maybe_set_exposure_time_us_(
      properties->exposure_time_us, last_known_settings_.exposure_time_us);

    last_known_settings_.binning =
      maybe_set_binning(properties->binning, last_known_settings_.binning);

    last_known_settings_.pixel_type = maybe_set_sample_type(
      properties->pixel_type, last_known_settings_.pixel_type);

    last_known_settings_.offset =
      maybe_set_offset(properties->offset, last_known_settings_.offset);

    last_known_settings_.shape =
      maybe_set_shape(properties->shape, last_known_settings_.shape);

    last_known_settings_.input_triggers.frame_start =
      maybe_set_input_trigger_frame_start(
        properties->input_triggers.frame_start,
        last_known_settings_.input_triggers.frame_start);

    last_known_settings_.output_triggers.exposure =
      maybe_set_output_trigger_exposure(
        properties->output_triggers.exposure,
        last_known_settings_.output_triggers.exposure);

    // TODO: consider calling get here
}

float
SpinnakerCamera::maybe_set_exposure_time_us_(float target_us,
                                             float last_value_us)
{
    if (target_us != last_value_us) {
        target_us = clamp(target_us, last_known_capabilities_.exposure_time_us.low, last_known_capabilities_.exposure_time_us.high);
        if (last_known_capabilities_.exposure_time_us.writable) {
            camera_->ExposureTime = target_us;
        }
    }
    return last_value_us;
}

uint8_t
SpinnakerCamera::maybe_set_binning(uint8_t target, uint8_t last_value)
{
    if (target != last_value) {
        target = clamp(target,
                       last_known_capabilities_.binning.low,
                       last_known_capabilities_.binning.high);
        if (last_known_capabilities_.binning.writable) {
            camera_->BinningHorizontal = target;
            camera_->BinningVertical = target;
        }
        return target;
    }
    return last_value;
}

SampleType
SpinnakerCamera::maybe_set_sample_type(SampleType target, SampleType last_known)
{
    CHECK(target < SampleTypeCount);
    if (target != last_known) {
        camera_->PixelFormat = to_pixel_format(target);
    }
    return target;
}

CameraProperties::camera_properties_offset_s
SpinnakerCamera::maybe_set_offset(
  CameraProperties::camera_properties_offset_s target,
  CameraProperties::camera_properties_offset_s last)
{
    if (target.x != last.x) {
        target.x = clamp(target.x,
                         last_known_capabilities_.offset.x.low,
                         last_known_capabilities_.offset.x.high);
        if (last_known_capabilities_.offset.x.writable) {
            camera_->OffsetX = target.x;
        }
        last.x = target.x;
    }
    if (target.y != last.y) {
        target.y = clamp(target.y,
                         last_known_capabilities_.offset.y.low,
                         last_known_capabilities_.offset.y.high);
        if (last_known_capabilities_.offset.y.writable) {
            camera_->OffsetY = target.y;
        }
        last.y = target.y;
    }
    return last;
}

CameraProperties::camera_properties_shape_s
SpinnakerCamera::maybe_set_shape(
  CameraProperties::camera_properties_shape_s target,
  CameraProperties::camera_properties_shape_s last)
{
    if (target.x != last.x) {
        target.x = clamp(target.x,
                         last_known_capabilities_.shape.x.low,
                         last_known_capabilities_.shape.x.high);
        if (last_known_capabilities_.shape.x.writable) {
            camera_->Width = target.x;
        }
        last.x = target.x;
    }
    if (target.y != last.y) {
        target.y = clamp(target.y,
                         last_known_capabilities_.shape.y.low,
                         last_known_capabilities_.shape.y.high);
        if (last_known_capabilities_.shape.y.writable) {
            camera_->Height = target.y;
        }
        last.y = target.y;
    }
    return last;
}

Trigger&
SpinnakerCamera::maybe_set_input_trigger_frame_start(Trigger& target,
                                                     const Trigger& last)
{
    if (!is_equal(target, last)) {
        if (IsReadable(camera_->TriggerSelector) &&
            IsWritable(camera_->TriggerSelector)) {
            // Always disable trigger before any other configuration.
            camera_->TriggerMode = "Off";

            camera_->TriggerSelector = "FrameStart";
            camera_->TriggerSource = to_trigger_source(target.line);
            camera_->TriggerActivation = to_trigger_activation(target.edge);
            camera_->TriggerMode = target.enable ? "On" : "Off";
        }
    }
    return target;
}

Trigger&
SpinnakerCamera::maybe_set_output_trigger_exposure(Trigger& target,
                                                   const Trigger& last)
{
    if (!is_equal(target, last)) {
        // switch IsReadable to a check
        if (IsReadable(camera_->LineSelector) &&
            IsWritable(camera_->LineSelector)) {
            camera_->LineSelector = "Line1";
            camera_->LineMode = "Output";
            camera_->LineSource = "ExposureActive";
        }
    }
    return target;
}

void
SpinnakerCamera::get(struct CameraProperties* properties)
{
    const std::scoped_lock lock(lock_);
    *properties = {
        .exposure_time_us = (float)camera_->ExposureTime(),
        .binning = (uint8_t)camera_->BinningHorizontal(),
        .pixel_type = to_sample_type(*(camera_->PixelFormat)),
        .offset = {
          .x = (uint32_t)camera_->OffsetX(),
          .y = (uint32_t)camera_->OffsetY(),
        },
        .shape = {
          .x = (uint32_t)camera_->Width(),
          .y = (uint32_t)camera_->Height(),
        },
    };

    // TODO: at least log when something cannot be populated

    // Only reads frame_start input trigger if it currently configured.
    if (IsReadable(camera_->TriggerSelector) && IsReadable(camera_->TriggerSource)) {
        if (*(camera_->TriggerSelector) == "FrameStart") {
            auto& trigger = properties->input_triggers.frame_start;
            trigger.kind = Signal_Input;
            trigger.enable = *(camera_->TriggerMode) == "On";
            trigger.line = to_trigger_line(*(camera_->TriggerSource));
            trigger.edge = to_trigger_edge(*(camera_->TriggerActivation));
        }
    }

    // Only reads exposure output trigger if it currently configured on line 1.
    if (IsReadable(camera_->LineSelector) && IsReadable(camera_->LineSource)) {
        if (*(camera_->LineSelector) == "Line1") {
            auto& trigger = properties->output_triggers.exposure;
            trigger.kind = Signal_Output;
            trigger.enable = *(camera_->LineSource) == "ExposureActive";
            trigger.line = 1;
            // TODO: check with Nathan if this is the expected edge.
            trigger.edge = TriggerEdge_LevelHigh;
        }
    }

    last_known_settings_ = *properties;
}

void
SpinnakerCamera::get_meta(struct CameraPropertyMetadata* meta) const
{
    const std::scoped_lock lock(lock_);
    query_exposure_time_capabilities_(meta);
    meta->line_interval_us = { .writable = false };
    meta->readout_direction = { .writable = false };
    query_binning_capabilities_(meta);
    query_roi_offset_capabilities_(meta);
    query_roi_shape_capabilities_(meta);
    query_pixel_type_capabilities_(meta);
    query_triggering_capabilities_(meta);
}

void
SpinnakerCamera::query_exposure_time_capabilities_(
  CameraPropertyMetadata* meta) const
{
    meta->exposure_time_us = {
        .writable = IsWritable(camera_->ExposureTime),
        .low = (float)camera_->ExposureTime.GetMin(),
        .high = (float)camera_->ExposureTime.GetMax(),
        .type = PropertyType_FloatingPrecision,
    };
}

void
SpinnakerCamera::query_binning_capabilities_(CameraPropertyMetadata* meta) const
{
    // Spinnaker supports independent horizontal and vertical binning.
    // Assume horizontal is representative for now.
    meta->binning = {
        .writable = IsWritable(camera_->BinningHorizontal),
        .low = (float)camera_->BinningHorizontal.GetMin(),
        .high = (float)camera_->BinningHorizontal.GetMax(),
        .type = PropertyType_FixedPrecision,
    };
}
void
SpinnakerCamera::query_roi_offset_capabilities_(
  CameraPropertyMetadata* meta) const
{
    meta->offset = {
        .x = {
          .writable = IsWritable(camera_->OffsetX),
          .low = (float)camera_->OffsetX.GetMin(),
          .high = (float)camera_->OffsetX.GetMax(),
          .type = PropertyType_FixedPrecision,
        },
        .y = {
          .writable = IsWritable(camera_->OffsetY),
          .low = (float)camera_->OffsetY.GetMin(),
          .high = (float)camera_->OffsetY.GetMax(),
          .type = PropertyType_FixedPrecision,
        },
    };
}
void
SpinnakerCamera::query_roi_shape_capabilities_(
  CameraPropertyMetadata* meta) const
{
    meta->shape = {
        .x = {
          .writable = IsWritable(camera_->Width),
          .low = (float)camera_->Width.GetMin(),
          .high = (float)camera_->Width.GetMax(),
          .type = PropertyType_FixedPrecision,
        },
        .y = {
          .writable = IsWritable(camera_->Height),
          .low = (float)camera_->Height.GetMin(),
          .high = (float)camera_->Height.GetMax(),
          .type = PropertyType_FixedPrecision,
        },
    };
}

void
SpinnakerCamera::query_pixel_type_capabilities_(
  CameraPropertyMetadata* meta) const
{
    meta->supported_pixel_types = 0;
    Spinnaker::GenApi::StringList_t pixel_formats;
    camera_->PixelFormat.GetSymbolics(pixel_formats);
    for (const Spinnaker::GenICam::gcstring format : pixel_formats) {
        const SampleType sample_type = to_sample_type(format);
        meta->supported_pixel_types |= (1ULL << sample_type);
    }
}

void
SpinnakerCamera::query_triggering_capabilities_(CameraPropertyMetadata* meta)
{
    // These are based on inspection of blackfly camera properties in SpinView.
    // ExposureActive can be selected using the Trigger Selector in SpinView,
    // but Spinnaker does not have a corresponding enum value in
    // TriggerSelectorEnums, so do not enable it as an input trigger.
    meta->digital_lines = {
        .line_count = 3,
        .names = {
          "Line0",
          "Line1",
          "Software",
        },
    };
    meta->triggers = {
        .exposure = { .input = 0, .output = 0b0010 },
        .frame_start = { .input = 0b0101, .output = 0 },
    };
}

void
SpinnakerCamera::get_shape(struct ImageShape* shape) const
{
    const std::scoped_lock lock(lock_);

    const uint32_t width = (int32_t)camera_->Width();
    const uint32_t height = (int32_t)camera_->Height();

    *shape = {
        .dims = {
            .channels = 1,
            .width = width,
            .height = height,
            .planes = 1,
        },
        .strides = {
          .channels = 1,
          .width = 1,
          .height = width,
          .planes = width*height,
        },
        .type = to_sample_type(*(camera_->PixelFormat)),
    };
}

void
SpinnakerCamera::start()
{
    const std::scoped_lock lock(lock_);
    frame_id_ = 0;

    EXPECT(IsWritable(camera_->AcquisitionMode), "Unable to set acquisition mode.");
    EXPECT(IsReadable(camera_->AcquisitionMode.GetEntryByName("Continuous")), "Unable to set acquisition mode to continuous.");
    camera_->AcquisitionMode = "Continuous";

    camera_->BeginAcquisition();
}

void
SpinnakerCamera::stop()
{
    const std::scoped_lock lock(lock_);
    // Disable the current trigger to prevent an effective deadlock between
    // EndAcquisition and GetNextFrame that is awaiting a trigger.
    // TODO: consider enabling triggers in start rather than configure because
    // otherwise disabling here creates some inconsistency.
    if (IsReadable(camera_->TriggerMode) && IsWritable(camera_->TriggerMode)) {
        camera_->TriggerMode = "Off";
    }
    if (camera_->IsStreaming()) {
        camera_->EndAcquisition();
    }
}

void
SpinnakerCamera::execute_trigger() const
{
    const std::scoped_lock lock(lock_);
    camera_->TriggerSoftware();
}

void
SpinnakerCamera::get_frame(void* im, size_t* nbytes, struct ImageInfo* info)
{
    // Adapted from the Acquisition.cpp example distributed with the Spinnaker
    // SDK. Cannot acquire the camera lock here because GetNextImage may await
    // a trigger indefinitely effectively causing a deadlock with other methods
    // that acquire the camera lock (like stop).
    Spinnaker::ImagePtr frame = camera_->GetNextImage();

    // No need to acquire the camera lock after getting the image because we
    // do not access the Spinnaker camera API from here (only the image) and
    // frame_id_ does not need to be guarded due to other ordering that
    // acquire guarantees.

    if (frame->IsIncomplete()) {
        LOGE(
          "Image incomplete: %s",
          Spinnaker::Image::GetImageStatusDescription(frame->GetImageStatus()));
    } else {
        const size_t width = frame->GetWidth();
        const size_t height = frame->GetHeight();
        const uint64_t timestamp_ns = frame->GetTimeStamp();

        EXPECT(frame->GetData(), "Expected non-null pointer");
        // The spinnaker default buffers may be aligned to certain byte
        // boundaries (e.g. 1024 for better performance over USB3), so
        // the buffer may be larger than acquire's, but not vice versa.
        EXPECT(*nbytes <= frame->GetBufferSize(),
               "Expected frame buffer size to be at least that allocated: %d",
               *nbytes);
        std::memcpy(im, frame->GetData(), *nbytes);

        *info = {
            .shape = {
                  .dims = { .channels = 1,
                            .width = (uint32_t)width,
                            .height = (uint32_t)height,
                            .planes = 1 },
                  .strides = { .channels = 1,
                               .width = 1,
                               .height = (int64_t)width,
                               .planes = (int64_t)(width * height),
                  },
                  .type = to_sample_type(frame->GetPixelFormatName()),
              },
              .hardware_timestamp = timestamp_ns,
              .hardware_frame_id = frame_id_++,
        };
    }

    // Possibly unneeded, but we manually release as instructed in the Spinnaker
    // Acquistion.cpp example because this image was retrieved directly from the camera.
    frame->Release();
}

//
// Driver declaration
//

struct SpinnakerDriver final : public Driver
{
    explicit SpinnakerDriver();
    ~SpinnakerDriver();

    uint32_t device_count();
    void describe(DeviceIdentifier* identifier, uint64_t i);
    void open(uint64_t device_id, struct Device** out);
    static void close(struct Device* in);
    void shutdown();

  private:
    // We need an external reference to the system pointer to keep it alive
    // even though it's a singleton. This should be initialized on construction
    // of the driver and manually released on shutdown as described in the
    // spinnaker docs.
    // http://softwareservices.flir.com/Spinnaker/latest/_programmer_guide.html#QuickSpin_API_and_Accessing_Camera_Parameters
    Spinnaker::SystemPtr system_;
};

//
// Non-throwing driver implementation
//

uint32_t
spinnakercam_device_count(struct Driver* self_)
{
    try {
        CHECK(self_);
        return ((struct SpinnakerDriver*)self_)->device_count();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return 0;
}

enum DeviceStatusCode
spinnakercam_describe(const struct Driver* self_,
                      struct DeviceIdentifier* identifier,
                      uint64_t i)
{
    try {
        CHECK(self_);
        ((struct SpinnakerDriver*)self_)->describe(identifier, i);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_open(struct Driver* self_, uint64_t device_id, struct Device** out)
{
    try {
        CHECK(self_);
        ((struct SpinnakerDriver*)self_)->open(device_id, out);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_close(struct Driver* self_, struct Device* in)
{
    try {
        CHECK(self_);
        ((struct SpinnakerDriver*)self_)->close(in);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

enum DeviceStatusCode
spinnakercam_shutdown(struct Driver* self_)
{
    try {
        CHECK(self_);
        delete (struct SpinnakerDriver*)self_;
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

//
// Driver implementation
//

SpinnakerDriver::SpinnakerDriver()
  : Driver{
      .device_count = ::spinnakercam_device_count,
      .describe = ::spinnakercam_describe,
      .open = ::spinnakercam_open,
      .close = ::spinnakercam_close,
      .shutdown = ::spinnakercam_shutdown,
  },
  system_(Spinnaker::System::GetInstance())
{
}

SpinnakerDriver::~SpinnakerDriver() {
    try {
        shutdown();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
}

void
SpinnakerDriver::describe(DeviceIdentifier* identifier, uint64_t i)
{
    check_spinnaker_camera_id(i);

    Spinnaker::CameraList camera_list = system_->GetCameras();
    Spinnaker::CameraPtr camera = camera_list.GetByIndex((unsigned int)i);
    Spinnaker::GenApi::INodeMap& nodeMap = camera->GetTLDeviceNodeMap();

    const Spinnaker::GenApi::CStringPtr vendor_name =
      nodeMap.GetNode("DeviceVendorName");
    const Spinnaker::GenApi::CStringPtr device_name =
      nodeMap.GetNode("DeviceModelName");
    const Spinnaker::GenApi::CStringPtr device_sn =
      nodeMap.GetNode("DeviceSerialNumber");

    *identifier = DeviceIdentifier{
        .device_id = (uint8_t)i,
        .kind = DeviceKind_Camera,
    };

    snprintf(identifier->name,
             sizeof(identifier->name),
             "%s %s %s",
             vendor_name->GetValue().c_str(),
             device_name->GetValue().c_str(),
             device_sn->GetValue().c_str());
}

uint32_t
SpinnakerDriver::device_count()
{
    return (uint32_t)system_->GetCameras().GetSize();
}

void
SpinnakerDriver::open(uint64_t device_id, struct Device** out)
{
    CHECK(out);
    check_spinnaker_camera_id(device_id);
    Spinnaker::CameraList camera_list = system_->GetCameras();
    Spinnaker::CameraPtr camera =
      camera_list.GetByIndex((unsigned int)device_id);
    *out = (Device*)new SpinnakerCamera(camera);
}

void
SpinnakerDriver::close(struct Device* in)
{
    CHECK(in);
    delete (SpinnakerCamera*)in;
}

void
SpinnakerDriver::shutdown()
{
    // Acquire needs shutdown to be idempotent, but System::ReleaseInstance
    // is not, so protect against a double release.
    if (system_.IsValid()) {
        // Possibly unneeded since this is a smart pointer, but this follows
        // the Spinnaker examples.
        system_->ReleaseInstance();
    }
}

} // end anonymous namespace

acquire_export struct Driver*
acquire_driver_init_v0(acquire_reporter_t reporter)
{
    try {
        logger_set_reporter(reporter);
        return new SpinnakerDriver();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return nullptr;
}
