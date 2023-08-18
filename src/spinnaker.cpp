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

TriggerEdge
to_trigger_edge(Spinnaker::TriggerActivationEnums activation)
{
    switch (activation) {
        case Spinnaker::TriggerActivation_RisingEdge:
            return TriggerEdge_Rising;
        case Spinnaker::TriggerActivation_FallingEdge:
            return TriggerEdge_Falling;
        case Spinnaker::TriggerActivation_AnyEdge:
            return TriggerEdge_AnyEdge;
        case Spinnaker::TriggerActivation_LevelHigh:
            return TriggerEdge_LevelHigh;
        case Spinnaker::TriggerActivation_LevelLow:
            return TriggerEdge_LevelLow;
        default:; // fall through to final return.
    }
    return TriggerEdge_Unknown;
}

Spinnaker::TriggerActivationEnums
to_trigger_activation(TriggerEdge edge)
{
    switch (edge) {
        case TriggerEdge_Rising:
            return Spinnaker::TriggerActivation_RisingEdge;
        case TriggerEdge_Falling:
            return Spinnaker::TriggerActivation_FallingEdge;
        case TriggerEdge_AnyEdge:
            return Spinnaker::TriggerActivation_AnyEdge;
        case TriggerEdge_LevelHigh:
            return Spinnaker::TriggerActivation_LevelHigh;
        case TriggerEdge_LevelLow:
            return Spinnaker::TriggerActivation_LevelLow;
        default:; // fall through to final error.
    }
    EXPECT(false, "Trigger edge %d unrecognized.", edge);
}

SampleType
to_sample_type(Spinnaker::PixelFormatEnums format)
{
    switch (format) {
        case Spinnaker::PixelFormat_Mono8:
            return SampleType_u8;
        case Spinnaker::PixelFormat_Mono8s:
            return SampleType_i8;
        case Spinnaker::PixelFormat_Mono10:
            return SampleType_u10;
        case Spinnaker::PixelFormat_Mono12:
            return SampleType_u12;
        case Spinnaker::PixelFormat_Mono14:
            return SampleType_u14;
        case Spinnaker::PixelFormat_Mono16:
            return SampleType_u16;
        case Spinnaker::PixelFormat_Mono16s:
            return SampleType_i16;
        case Spinnaker::PixelFormat_Mono32f:
            return SampleType_f32;
        default:; // fall through to final return.
    }
    return SampleType_Unknown;
}

Spinnaker::PixelFormatEnums
to_pixel_format(SampleType type)
{
    switch (type) {
        case SampleType_u8:
            return Spinnaker::PixelFormat_Mono8;
        case SampleType_i8:
            return Spinnaker::PixelFormat_Mono8s;
        case SampleType_u10:
            return Spinnaker::PixelFormat_Mono10;
        case SampleType_u12:
            return Spinnaker::PixelFormat_Mono12;
        case SampleType_u14:
            return Spinnaker::PixelFormat_Mono14;
        case SampleType_u16:
            return Spinnaker::PixelFormat_Mono16;
        case SampleType_i16:
            return Spinnaker::PixelFormat_Mono16s;
        case SampleType_f32:
            return Spinnaker::PixelFormat_Mono32f;
        default:; // fall through to final error.
    }
    EXPECT(false, "Sample type %d unrecognized.", type);
}

// Define the software line as acquire's line 2 because the Blackfly USB3 camera
// only has two physical lines (0 and 1).
// Similarly define acquire's line 3 as unknown.

uint8_t
to_trigger_line(Spinnaker::TriggerSourceEnums source)
{
    switch (source) {
        case Spinnaker::TriggerSource_Line0:
            return 0;
        case Spinnaker::TriggerSource_Line1:
            return 1;
        case Spinnaker::TriggerSource_Software:
            return 2;
        default:; // fall through to final return.
    }
    return 3;
}

Spinnaker::TriggerSourceEnums
to_trigger_source(uint8_t line)
{
    switch (line) {
        case 0:
            return Spinnaker::TriggerSource_Line0;
        case 1:
            return Spinnaker::TriggerSource_Line1;
        case 2:
            return Spinnaker::TriggerSource_Software;
        default:; // fall through to final error.
    }
    EXPECT(false, "Trigger line %d unrecognized.", line);
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
    // If things didn't get shut down properly before,
    // sometimes start fails.
    for (int attempts = 0; attempts < 2; ++attempts) {
        try {
            CHECK(self_);
            ((struct SpinnakerCamera*)self_)->start();
            return Device_Ok;
        } catch (const std::exception& exc) {
            LOGE("Exception: %s\n", exc.what());
        } catch (...) {
            LOGE("Exception: (unknown)");
        }
        LOGE("Retrying camera start");
        spinnakercam_stop(self_);
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
    // double check if isvalid is for point or for camera
    CHECK(camera->IsValid());
    // this should prob go too
    stop();
    // This could happen if the same device is selected on two streams
    // TODO: strongly consider using EXPECT to check this is not initialized
    if (!camera->IsInitialized()) {
        camera->Init();
    }
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
    if (target == last_known) {
        return last_known;
    }
    const Spinnaker::PixelFormatEnums format = to_pixel_format(target);
    if (IsReadable(camera_->PixelFormat) && IsWritable(camera_->PixelFormat)) {
        Spinnaker::GenApi::IEnumEntry* entry =
          camera_->PixelFormat.GetEntry((int)format);
        EXPECT(IsReadable(entry),
               "Sample type %d recognized as pixel format %d, but not "
               "supported by this camera.",
               target,
               format);
        camera_->PixelFormat.SetIntValue(entry->GetValue());
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
    if (is_equal(target, last)) {
        return target;
    }
    if (IsReadable(camera_->TriggerSelector) &&
        IsWritable(camera_->TriggerSelector)) {
        // Always disable trigger before any other configuration.
        camera_->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);

        camera_->TriggerSelector.SetValue(
          Spinnaker::TriggerSelector_FrameStart);

        const Spinnaker::TriggerSourceEnums trigger_source =
          to_trigger_source(target.line);
        camera_->TriggerSource.SetValue(trigger_source);

        const Spinnaker::TriggerActivationEnums trigger_activation =
          to_trigger_activation(target.edge);
        camera_->TriggerActivation.SetValue(trigger_activation);

        camera_->TriggerMode.SetValue(target.enable
                                        ? Spinnaker::TriggerMode_On
                                        : Spinnaker::TriggerMode_Off);
    }
    return target;
}

Trigger&
SpinnakerCamera::maybe_set_output_trigger_exposure(Trigger& target,
                                                   const Trigger& last)
{
    if (is_equal(target, last)) {
        return target;
    }
    // switch IsReadable to a check
    if (IsReadable(camera_->LineSelector) &&
        IsWritable(camera_->LineSelector)) {
        camera_->LineSelector.SetValue(Spinnaker::LineSelector_Line1);
        camera_->LineMode.SetValue(Spinnaker::LineMode_Output);
        camera_->LineSource.SetValue(Spinnaker::LineSource_ExposureActive);
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
        .pixel_type = to_sample_type(camera_->PixelFormat()),
        .offset = {
          .x = (uint32_t)camera_->OffsetX(),
          .y = (uint32_t)camera_->OffsetY(),
        },
        .shape = {
          .x = (uint32_t)camera_->Width(),
          .y = (uint32_t)camera_->Height(),
        },
    };

    // at least log when something cannot be populated

    // Only reads frame_start input trigger if it currently configured.
    if (IsReadable(camera_->TriggerSelector) &&
        IsReadable(camera_->TriggerSource)) {
        if (camera_->TriggerSelector() ==
            Spinnaker::TriggerSelector_FrameStart) {
            auto& trigger = properties->input_triggers.frame_start;
            trigger.kind = Signal_Input;
            trigger.enable =
              camera_->TriggerMode() == Spinnaker::TriggerMode_On;
            trigger.line = to_trigger_line(camera_->TriggerSource());
            trigger.edge = to_trigger_edge(camera_->TriggerActivation());
        }
    }

    // Only reads exposure output trigger if it currently configured on line 1.
    if (IsReadable(camera_->LineSelector) && IsReadable(camera_->LineSource)) {
        if (camera_->LineSelector() == Spinnaker::LineSelector_Line1) {
            auto& trigger = properties->output_triggers.exposure;
            trigger.kind = Signal_Output;
            trigger.enable =
              camera_->LineSource() == Spinnaker::LineSource_ExposureActive;
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
    Spinnaker::GenApi::NodeList_t pixel_formats;
    camera_->PixelFormat.GetEntries(pixel_formats);
    for (Spinnaker::GenApi::INode* node : pixel_formats) {
        auto entry = dynamic_cast<Spinnaker::GenApi::IEnumEntry*>(node);
        EXPECT(entry, "Unable to cast to enum entry.");
        const auto format = (Spinnaker::PixelFormatEnums)entry->GetValue();
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
        .type = to_sample_type(camera_->PixelFormat()),
    };
}

void
SpinnakerCamera::start()
{
    const std::scoped_lock lock(lock_);
    frame_id_ = 0;

    EXPECT(IsReadable(camera_->AcquisitionMode) &&
             IsWritable(camera_->AcquisitionMode),
           "Unable to get and set acquisition mode.");
    EXPECT(IsReadable(camera_->AcquisitionMode.GetEntry(
             (int64_t)Spinnaker::AcquisitionMode_Continuous)),
           "Unable to get or set acquisition mode to continuous.");
    camera_->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

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
        camera_->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
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
    // a trigger indefinitely effectively causing a deadlock that is especially
    // bad for stop since it prevents teardown of the camera.
    Spinnaker::ImagePtr frame = camera_->GetNextImage();

    // TODO: Consider explaining why we don't acquire the camera mutex here.

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
                  .type = to_sample_type(frame->GetPixelFormat()),
              },
              .hardware_timestamp = timestamp_ns,
              // TODO: explain why not mutex here either
              .hardware_frame_id = frame_id_++,
        };
    }

    frame->Release();
}

//
// Driver declaration
//

// TODO: why is this not private inheritance and assignment like Camera?
struct SpinnakerDriver final : public Driver
{
    SpinnakerDriver();
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
    // True if shutdown has been called at least once on this, false otherwise.
    bool is_shutdown_;
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
  system_(Spinnaker::System::GetInstance()),
  is_shutdown_(false)
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
    // Acquire needs shutdown to be idempotent, but unclear if spinnaker's
    // System::ReleaseInstance is, so protect against a double release.
    // TODO: could we use system_.IsValid() instead?
    if (!is_shutdown_) {
        is_shutdown_ = true;
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
