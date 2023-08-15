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

template<typename K, typename V>
std::unordered_map<V, K>
invert_map(const std::unordered_map<K, V>& map)
{
    std::unordered_map<V, K> inverse;
    for (const auto& [k, v] : map) {
        inverse[v] = k;
    }
    return inverse;
}

const std::unordered_map<Spinnaker::TriggerActivationEnums, TriggerEdge>
  trig_edge_table{
      { Spinnaker::TriggerActivation_RisingEdge, TriggerEdge_Rising },
      { Spinnaker::TriggerActivation_FallingEdge, TriggerEdge_Falling },
      { Spinnaker::TriggerActivation_AnyEdge, TriggerEdge_AnyEdge },
      { Spinnaker::TriggerActivation_LevelHigh, TriggerEdge_LevelHigh },
      { Spinnaker::TriggerActivation_LevelLow, TriggerEdge_LevelLow },
  };

const std::unordered_map<TriggerEdge, Spinnaker::TriggerActivationEnums>
  trig_edge_inv_table = invert_map(trig_edge_table);

const std::unordered_map<Spinnaker::PixelFormatEnums, SampleType>
  sample_type_table{
      { Spinnaker::PixelFormat_Mono8, SampleType_u8 },
      { Spinnaker::PixelFormat_Mono8s, SampleType_i8 },
      { Spinnaker::PixelFormat_Mono10, SampleType_u10 },
      { Spinnaker::PixelFormat_Mono12, SampleType_u12 },
      { Spinnaker::PixelFormat_Mono14, SampleType_u14 },
      { Spinnaker::PixelFormat_Mono16, SampleType_u16 },
      { Spinnaker::PixelFormat_Mono16s, SampleType_i16 },
      { Spinnaker::PixelFormat_Mono32f, SampleType_f32 },
  };

const std::unordered_map<SampleType, Spinnaker::PixelFormatEnums>
  sample_type_inv_table = invert_map(sample_type_table);

// Arbitrarily defines the software line to be acquire's digital line number 2.
enum TriggerLine
{
    TriggerLine_Line0 = 0,
    TriggerLine_Line1 = 1,
    TriggerLine_Software = 2,
    TriggerLine_Unknown = 3
};

const std::unordered_map<Spinnaker::TriggerSourceEnums, TriggerLine>
  trig_line_table{
      { Spinnaker::TriggerSource_Line0, TriggerLine_Line0 },
      { Spinnaker::TriggerSource_Line1, TriggerLine_Line1 },
      { Spinnaker::TriggerSource_Software, TriggerLine_Software },
  };

const std::unordered_map<TriggerLine, Spinnaker::TriggerSourceEnums>
  trig_line_inv_table = invert_map(trig_line_table);

template<typename K, typename V>
V
at_or(const std::unordered_map<K, V>& table, const K& key, V dflt)
{
    const auto it = table.find(key);
    if (it == std::end(table)) {
        return dflt;
    }
    return it->second;
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
    // Guards access to camera_
    mutable std::mutex lock_;
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
    CHECK(camera->IsValid());
    stop();
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
    } catch (...) {
        ;
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
}

float
SpinnakerCamera::maybe_set_exposure_time_us_(float target_us,
                                             float last_value_us)
{
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
            camera_->BinningHorizontal.SetValue(target);
            camera_->BinningVertical.SetValue(target);
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
    const Spinnaker::PixelFormatEnums format = sample_type_inv_table.at(target);
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
          trig_line_inv_table.at((TriggerLine)target.line);
        camera_->TriggerSource.SetValue(trigger_source);

        const Spinnaker::TriggerActivationEnums trigger_activation =
          trig_edge_inv_table.at(target.edge);
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
        .exposure_time_us = (float)camera_->ExposureTime.GetValue(),
        .binning = (uint8_t)camera_->BinningHorizontal.GetValue(),
        .pixel_type = at_or(sample_type_table, camera_->PixelFormat(), SampleType_Unknown),
        .offset = {
          .x = (uint32_t)camera_->OffsetX.GetValue(),
          .y = (uint32_t)camera_->OffsetY.GetValue(),
        },
        .shape = {
          .x = (uint32_t)camera_->Width.GetValue(),
          .y = (uint32_t)camera_->Height.GetValue(),
        },
    };

    // Only reads frame_start input trigger if it currently configured.
    if (IsReadable(camera_->TriggerSelector)) {
        if (camera_->TriggerSelector() ==
            Spinnaker::TriggerSelector_FrameStart) {
            auto& trigger = properties->input_triggers.frame_start;
            trigger.kind = Signal_Input;
            trigger.enable =
              camera_->TriggerMode() == Spinnaker::TriggerMode_On;
            trigger.line = at_or(
              trig_line_table, camera_->TriggerSource(), TriggerLine_Unknown);
            trigger.edge = at_or(trig_edge_table,
                                 camera_->TriggerActivation(),
                                 TriggerEdge_Unknown);
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
        const SampleType sample_type =
          at_or(sample_type_table, format, SampleType_Unknown);
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

    const uint32_t width = (int32_t)camera_->Width.GetValue();
    const uint32_t height = (int32_t)camera_->Height.GetValue();

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
        .type = at_or(sample_type_table, camera_->PixelFormat(), SampleType_Unknown),
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
                  .type = at_or(sample_type_table, frame->GetPixelFormat(), SampleType_Unknown),
              },
              .hardware_timestamp = timestamp_ns,
              // TODO: should frame_id be incremented before GetNextImage?
              .hardware_frame_id = frame_id_++,
        };
    }

    frame->Release();
}

//
// Driver declaration
//

struct SpinnakerDriver final : public Driver
{
    SpinnakerDriver(Spinnaker::SystemPtr system);

    uint32_t device_count();
    void describe(DeviceIdentifier* identifier, uint64_t i);
    void open(uint64_t device_id, struct Device** out);
    static void close(struct Device* in);

  private:
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
spinnakercam_shutdown_(struct Driver* self_)
{
    try {
        CHECK(self_);
        delete (SpinnakerDriver*)self_;
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

SpinnakerDriver::SpinnakerDriver(Spinnaker::SystemPtr system)
  : Driver{
      .device_count = ::spinnakercam_device_count,
      .describe = ::spinnakercam_describe,
      .open = ::spinnakercam_open,
      .close = ::spinnakercam_close,
      .shutdown = ::spinnakercam_shutdown_,
  },
  system_(system)
{
}

void
SpinnakerDriver::describe(DeviceIdentifier* identifier, uint64_t i)
{
    // TODO: shouldn't this check be done earlier?
    // DeviceManager device_id expects a uint8
    EXPECT(i < (1 << 8), "Expected a uint8 device index. Got: %llu", i);

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
    EXPECT(device_id < (1ULL << 8 * sizeof(int)) - 1,
           "Expected an int32 device id. Got: %llu",
           device_id);
    Spinnaker::CameraList camera_list = system_->GetCameras();
    Spinnaker::CameraPtr camera =
      camera_list.GetByIndex((unsigned int)device_id);
    *out = (Device*)new SpinnakerCamera(camera);
}

void
SpinnakerDriver::close(struct Device* in)
{
    CHECK(in);
    auto camera = (SpinnakerCamera*)in;
    delete camera;
    // TODO: should proably release the system instance here,
    // but this is static and calling GetInstance multiple times
    // does not behave as one would expect with a singleton.
}

} // end anonymous namespace

acquire_export struct Driver*
acquire_driver_init_v0(acquire_reporter_t reporter)
{
    try {
        logger_set_reporter(reporter);
        // Not passing through system to the camera causes an exception
        // to be thrown when the system pointer goes out of scope at the
        // end of this function. Would be good to understand this better.
        Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
        return new SpinnakerDriver(system);
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return nullptr;
}
