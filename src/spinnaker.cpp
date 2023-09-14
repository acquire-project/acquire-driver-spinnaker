/// @file Driver wrapping the Spinnaker SDK.
///
/// This implements SpinnakerCamera and SpinnakerDriver structs in C++
/// since the Spinnaker SDK is C++. The methods of those classes are
/// then bound to the Camera and Driver C structs in the Acquire API.

#include "device/props/camera.h"
#include "device/kit/camera.h"
#include "device/kit/driver.h"
#include "platform.h"
#include "logger.h"

#include "CameraDefs.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <algorithm>
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

// Define GenICam enum member names as constant strings to avoid typos
// and to avoid temporary gcstring instances when used with Spinnaker.
const Spinnaker::GenICam::gcstring genicam_off("Off");
const Spinnaker::GenICam::gcstring genicam_on("On");
const Spinnaker::GenICam::gcstring genicam_acquisition_start(
  "AcquisitionStart");
const Spinnaker::GenICam::gcstring genicam_frame_start("FrameStart");
const Spinnaker::GenICam::gcstring genicam_line_1("Line1");
const Spinnaker::GenICam::gcstring genicam_software("Software");
const Spinnaker::GenICam::gcstring genicam_input("Input");
const Spinnaker::GenICam::gcstring genicam_output("Output");
const Spinnaker::GenICam::gcstring genicam_exposure_active("ExposureActive");
const Spinnaker::GenICam::gcstring genicam_trigger_wait("FrameTriggerWait");
const Spinnaker::GenICam::gcstring genicam_user_output_1("UserOutput1");
const Spinnaker::GenICam::gcstring genicam_user_output_value_1(
  "UserOutputValue1");
const Spinnaker::GenICam::gcstring genicam_continuous("Continuous");
const Spinnaker::GenICam::gcstring genicam_timed("Timed");

// Inverse map lookup that returns a default key if the value is not found.
template<typename T>
T
inv_at_or(const std::unordered_map<T, Spinnaker::GenICam::gcstring>& table,
          const Spinnaker::GenICam::gcstring& value,
          const T default_key)
{
    for (const auto& [k, v] : table) {
        if (v == value) {
            return k;
        }
    }
    return default_key;
}

// Maps Acquire trigger edges to GenICam activation strings.
const std::unordered_map<TriggerEdge, Spinnaker::GenICam::gcstring>
  trigger_edge_to_activation{
      { TriggerEdge_Rising, "RisingEdge" },
      { TriggerEdge_Falling, "FallingEdge" },
      { TriggerEdge_AnyEdge, "AnyEdge" },
      { TriggerEdge_LevelHigh, "LevelHigh" },
      { TriggerEdge_LevelLow, "LevelLow" },
  };

TriggerEdge
to_trigger_edge(const Spinnaker::GenICam::gcstring& activation)
{
    return inv_at_or(
      trigger_edge_to_activation, activation, TriggerEdge_Unknown);
}

const Spinnaker::GenICam::gcstring&
to_trigger_activation(TriggerEdge edge)
{
    return trigger_edge_to_activation.at(edge);
}

// Maps Acquire sample types to GenICam pixel format strings.
const std::unordered_map<SampleType, Spinnaker::GenICam::gcstring>
  sample_type_to_pixel_format{
      { SampleType_u8, "Mono8" },    { SampleType_i8, "Mono8s" },
      { SampleType_u10, "Mono10" },  { SampleType_u12, "Mono12" },
      { SampleType_u14, "Mono14" },  { SampleType_u16, "Mono16" },
      { SampleType_i16, "Mono16s" }, { SampleType_f32, "Mono32f" },
  };

SampleType
to_sample_type(const Spinnaker::GenICam::gcstring& pixel_format)
{
    return inv_at_or(
      sample_type_to_pixel_format, pixel_format, SampleType_Unknown);
}

const Spinnaker::GenICam::gcstring&
to_pixel_format(SampleType sample_type)
{
    return sample_type_to_pixel_format.at(sample_type);
}

// Maps Acquire line numbers to GenICam line source strings.
// Define the software line as acquire's line 7 to leave lines
// 0-6 to map directly to the Oryx camera's lines.
const std::unordered_map<uint8_t, Spinnaker::GenICam::gcstring>
  trigger_line_to_source{
      { 0, "Line0" }, { 1, "Line1" }, { 2, "Line2" }, { 3, "Line3" },
      { 4, "Line4" }, { 5, "Line5" }, { 6, "Line6" }, { 7, "Software" },
  };

uint8_t
to_trigger_line(const Spinnaker::GenICam::gcstring& source)
{
    // Acquire's line 8 is unassigned here so use it for an unknown line.
    return inv_at_or<uint8_t>(trigger_line_to_source, source, 8);
}

const Spinnaker::GenICam::gcstring&
to_trigger_source(uint8_t line)
{
    return trigger_line_to_source.at(line);
}

// Returns true if a Spinnaker node is writable, false otherwise.
bool
check_node_writable(Spinnaker::GenApi::INode& node)
{
    if (!Spinnaker::GenApi::IsWritable(node)) {
        LOGE("%s is not writable.", node.GetName().c_str());
        return false;
    }
    return true;
}

// Various convenience Spinnaker node value setters.
void
set_enum_node(Spinnaker::GenApi::IEnumeration& node,
              const Spinnaker::GenICam::gcstring& value)
{
    if (check_node_writable(node)) {
        node = value;
    }
}

void
set_bool_node(Spinnaker::GenApi::IBoolean& node, bool value)
{
    if (check_node_writable(node)) {
        node = value;
    }
}

void
set_int_node(Spinnaker::GenApi::IInteger& node, int64_t value)
{
    if (check_node_writable(node)) {
        const int64_t min = node.GetMin();
        const int64_t max = node.GetMax();
        value = std::clamp(value, min, max);
        node = value;
    }
}

void
set_float_node(Spinnaker::GenApi::IFloat& node, double value)
{
    if (check_node_writable(node)) {
        const double min = node.GetMin();
        const double max = node.GetMax();
        value = std::clamp(value, min, max);
        node = value;
    }
}

// Returns true if the given name is a valid entry for the given enum node,
// false otherwise.
bool
is_enum_name(Spinnaker::GenApi::IEnumeration& node,
             const Spinnaker::GenICam::gcstring& name)
{
    return IsReadable(node.GetEntryByName(name));
}

// Returns a representative Spinnaker binning node.
// Spinnaker supports independent horizontal and vertical binning, but
// Acquire only supports one binning value.
Spinnaker::GenApi::IInteger&
get_binning_node(const Spinnaker::CameraPtr& camera)
{
    // Favor a writable node if present.
    Spinnaker::GenApi::IInteger& binning = camera->BinningVertical;
    if (!IsWritable(camera->BinningVertical) &&
        IsWritable(camera->BinningHorizontal)) {
        binning = camera->BinningHorizontal;
    }
    return binning;
}

// Check that a cast from Acquire's camera ID is appropriate for Spinnaker.
void
check_spinnaker_camera_id(uint64_t id)
{
    constexpr uint64_t limit = std::numeric_limits<unsigned int>::max();
    EXPECT(id < limit, "Expected an unsigned int device index. Got: %llu", id);
}

// Some utilities for updating input and output triggers/lines in place
// based on the current state of the camera.

template<typename T>
Trigger*
find_first_enabled_trigger(T& triggers)
{
    size_t n_triggers = sizeof(triggers) / sizeof(struct Trigger);
    auto trigger = (struct Trigger*)&triggers;
    for (int i = 0; i < n_triggers; ++i) {
        if (trigger[i].enable)
            return trigger + i;
    }
    return nullptr;
}

void
update_input_trigger(Spinnaker::CameraPtr& camera, Trigger& trigger)
{
    trigger.kind = Signal_Input;
    trigger.enable = true;
    trigger.line = to_trigger_line(*(camera->TriggerSource));
    // TriggerActivation can be non-readable when using the Software
    // trigger for some cameras (e.g. Oryx).
    trigger.edge = IsReadable(camera->TriggerActivation)
                     ? to_trigger_edge(*(camera->TriggerActivation))
                     : TriggerEdge_Unknown;
}

void
update_output_trigger(Spinnaker::CameraPtr& camera, Trigger& trigger)
{
    trigger.kind = Signal_Output;
    trigger.enable = true;
    trigger.line = to_trigger_line(*(camera->LineSelector));
    trigger.edge =
      camera->LineInverter() ? TriggerEdge_LevelLow : TriggerEdge_LevelHigh;
}

void
set_input_trigger(Spinnaker::CameraPtr& camera,
                  const Trigger* trigger,
                  const Spinnaker::GenICam::gcstring& name)
{
    if (is_enum_name(camera->TriggerSelector, name)) {
        set_enum_node(camera->TriggerSelector, name);
        set_enum_node(camera->TriggerSource, to_trigger_source(trigger->line));
        // TriggerActivation can be non-writable when using the Software trigger
        // for some cameras (e.g. Oryx).
        if (IsWritable(camera->TriggerActivation)) {
            set_enum_node(camera->TriggerActivation,
                          to_trigger_activation(trigger->edge));
        }
        set_enum_node(camera->TriggerMode,
                      trigger->enable ? genicam_on : genicam_off);
    }
}

void
set_output_trigger(Spinnaker::CameraPtr& camera,
                   const Trigger& trigger,
                   const Spinnaker::GenICam::gcstring& name)
{
    if (trigger.enable && is_enum_name(camera->LineSource, name)) {
        set_enum_node(camera->LineSelector, to_trigger_source(trigger.line));
        set_enum_node(camera->LineMode, genicam_output);
        set_enum_node(camera->LineSource, name);
    }
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
    void get_meta(struct CameraPropertyMetadata* meta);
    void get_shape(struct ImageShape* shape) const;
    void start();
    void stop();
    void execute_trigger() const;
    void get_frame(void* im, size_t* nbytes, struct ImageInfo* info);

  private:
    // Guards access to all private state.
    mutable std::mutex lock_;
    // Used as a concise way to get and set a Spinnaker camera's state,
    // rather than through its GenICam node map.
    Spinnaker::CameraPtr camera_;
    // True if the camera has been started, false otherwise.
    bool started_;
    // Setting properties on the device may be expensive, so these are
    // used to avoid doing so when a value is unchanged.
    struct CameraProperties last_known_settings_;

    void query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const;
    void query_binning_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_offset_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_shape_capabilities_(CameraPropertyMetadata* meta) const;
    void query_pixel_type_capabilities_(CameraPropertyMetadata* meta) const;
    void query_triggering_capabilities_(CameraPropertyMetadata* meta) const;

    void maybe_set_roi_(uint8_t binning,
                        CameraProperties::camera_properties_offset_s offset,
                        CameraProperties::camera_properties_shape_s shape);
    void maybe_set_sample_type_(SampleType target);
    void maybe_set_exposure_time_us_(float target_us);
    void maybe_set_input_trigger_(struct CameraProperties* properties);
    void maybe_set_output_triggers_(struct CameraProperties* properties);
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
  , started_(false)
{
    camera->Init();
    get(&last_known_settings_);
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
    maybe_set_roi_(properties->binning, properties->offset, properties->shape);
    maybe_set_sample_type_(properties->pixel_type);
    maybe_set_input_trigger_(properties);
    maybe_set_output_triggers_(properties);

    // Setting exposure time may not be compatible with certain triggers
    // (e.g. ExposureActive), so set it last.
    if (find_first_enabled_trigger(properties->input_triggers) !=
        &properties->input_triggers.exposure) {
        maybe_set_exposure_time_us_(properties->exposure_time_us);
    }
}

void
SpinnakerCamera::maybe_set_roi_(
  uint8_t binning,
  CameraProperties::camera_properties_offset_s offset,
  CameraProperties::camera_properties_shape_s shape)
{
    const uint8_t last_binning = last_known_settings_.binning;
    CameraProperties::camera_properties_offset_s& last_offset =
      last_known_settings_.offset;
    CameraProperties::camera_properties_shape_s& last_shape =
      last_known_settings_.shape;

    // If nothing changed, then nothing should be set, so return early.
    if (binning == last_binning && offset.x == last_offset.x &&
        offset.y == last_offset.y && shape.x == last_shape.x &&
        shape.y == last_shape.y) {
        return;
    }

    // Otherwise check if the image region is entirely on the sensor.
    // If it's not, then we log the error and return.
    const int64_t right = binning * (offset.x + shape.x);
    const int64_t bottom = binning * (offset.y + shape.y);
    if (right > camera_->SensorWidth()) {
        LOGE("Image region is too wide. Rejecting set request.");
        return;
    }
    if (bottom > camera_->SensorHeight()) {
        LOGE("Image region is too tall. Rejecting set request.");
        return;
    }

    // Even if the requested image region is valid, the current offset may
    // prevent us from changing the binning or shape value.
    // Since a 0 offset should always be valid regardless of the current
    // binning or shape value, reset it to 0 before changing anything else.
    if (last_offset.x != 0) {
        camera_->OffsetX = 0;
    }
    if (last_offset.y != 0) {
        camera_->OffsetY = 0;
    }

    if (binning != last_binning) {
        const bool supports_horizontal_binning =
          IsWritable(camera_->BinningHorizontal);
        const bool supports_vertical_binning =
          IsWritable(camera_->BinningVertical);

        // Some cameras only support setting either horizontal or vertical.
        if (supports_horizontal_binning) {
            set_int_node(camera_->BinningHorizontal, (int64_t)binning);
        }
        if (supports_vertical_binning) {
            set_int_node(camera_->BinningVertical, (int64_t)binning);
        }
        if (!(supports_horizontal_binning || supports_vertical_binning)) {
            LOGE("Neither horizontal nor vertical binning is writable.");
        }
        Spinnaker::GenApi::IInteger& node = get_binning_node(camera_);
        last_known_settings_.binning = (uint8_t)node();
    }

    if (shape.x != last_shape.x) {
        set_int_node(camera_->Width, (int64_t)shape.x);
        last_shape.x = (uint32_t)camera_->Width();
    }
    if (shape.y != last_shape.y) {
        set_int_node(camera_->Height, (int64_t)shape.y);
        last_shape.y = (uint32_t)camera_->Height();
    }

    if (offset.x != 0) {
        set_int_node(camera_->OffsetX, (int64_t)offset.x);
        last_offset.x = (uint32_t)camera_->OffsetX();
    }
    if (offset.y != 0) {
        set_int_node(camera_->OffsetY, (int64_t)offset.y);
        last_offset.y = (uint32_t)camera_->OffsetY();
    }
}

void
SpinnakerCamera::maybe_set_sample_type_(SampleType target)
{
    CHECK(target < SampleTypeCount);
    if (target != last_known_settings_.pixel_type) {
        set_enum_node(camera_->PixelFormat, to_pixel_format(target));
        last_known_settings_.pixel_type =
          to_sample_type(*(camera_->PixelFormat));
    }
}

void
SpinnakerCamera::maybe_set_exposure_time_us_(float target_us)
{
    // Always set the exposure mode because enabling ExposureActive
    // may automatically switch this to be TriggerWidth.
    set_enum_node(camera_->ExposureMode, genicam_timed);
    set_enum_node(camera_->ExposureAuto, genicam_off);

    if (target_us != last_known_settings_.exposure_time_us) {
        set_float_node(camera_->ExposureTime, (double)target_us);
        last_known_settings_.exposure_time_us = (float)camera_->ExposureTime();
    }
}

void
SpinnakerCamera::maybe_set_input_trigger_(struct CameraProperties* properties)
{
    // First turn triggering off before any other configuration as in
    // Spinnaker's Trigger.cpp example.
    set_enum_node(camera_->TriggerMode, genicam_off);

    // Then disable all triggering.
    Spinnaker::GenApi::StringList_t selectors;
    camera_->TriggerSelector.GetSymbolics(selectors);
    for (const auto& selector : selectors) {
        set_enum_node(camera_->TriggerSelector, selector);
        set_enum_node(camera_->TriggerMode, genicam_off);
    }

    // Finally, use the first trigger enabled in acquire, if any.
    auto& input_triggers = properties->input_triggers;
    Trigger* trigger = find_first_enabled_trigger(input_triggers);
    if ((trigger == &input_triggers.acquisition_start)) {
        set_input_trigger(camera_, trigger, genicam_acquisition_start);
    } else if ((trigger == &input_triggers.frame_start)) {
        set_input_trigger(camera_, trigger, genicam_frame_start);
    } else if ((trigger == &input_triggers.exposure)) {
        set_input_trigger(camera_, trigger, genicam_exposure_active);
    } else if (trigger != nullptr) {
        LOGE("Found an unrecognized trigger.");
    }
}

void
SpinnakerCamera::maybe_set_output_triggers_(struct CameraProperties* properties)
{
    auto& triggers = properties->output_triggers;
    set_output_trigger(camera_, triggers.exposure, genicam_exposure_active);
    set_output_trigger(camera_, triggers.frame_start, genicam_frame_start);
    set_output_trigger(camera_, triggers.trigger_wait, genicam_trigger_wait);
}

void
SpinnakerCamera::get(struct CameraProperties* properties)
{
    const std::scoped_lock lock(lock_);
    Spinnaker::GenApi::IInteger& binning = get_binning_node(camera_);
    *properties = {
        .exposure_time_us = (float)camera_->ExposureTime(),
        .binning = (uint8_t)binning(),
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

    // Zero out all the input triggers, then only update if the currently
    // selected trigger is enabled.
    memset(&properties->input_triggers, 0, sizeof(properties->input_triggers));
    if (*(camera_->TriggerMode) == genicam_on) {
        const Spinnaker::GenICam::gcstring trigger =
          *(camera_->TriggerSelector);
        if (trigger == genicam_acquisition_start) {
            update_input_trigger(camera_,
                                 properties->input_triggers.acquisition_start);
        } else if (trigger == genicam_frame_start) {
            update_input_trigger(camera_,
                                 properties->input_triggers.frame_start);
        } else if (trigger == genicam_exposure_active) {
            update_input_trigger(camera_, properties->input_triggers.exposure);
        } else {
            LOGE("Found an unrecognized trigger.");
        }
    }

    // Zero out all the output triggers, then iterate over all the lines,
    // allowing multiple output triggers to be enabled for Acquire.
    memset(
      &properties->output_triggers, 0, sizeof(properties->output_triggers));
    Spinnaker::GenApi::StringList_t lines;
    camera_->LineSelector.GetSymbolics(lines);
    auto& triggers = properties->output_triggers;
    for (auto& line : lines) {
        camera_->LineSelector = line;
        // Some lines do not have a readable LineMode (e.g. Oryx Line 6), so
        // skip them.
        if (!IsReadable(camera_->LineMode) ||
            *(camera_->LineMode) != genicam_output) {
            continue;
        }
        // Spinnaker allows us to use the same source for multiple output
        // lines, so only enable each trigger for the first line found.
        const Spinnaker::GenICam::gcstring source = *(camera_->LineSource);
        if (!triggers.exposure.enable && source == genicam_exposure_active) {
            update_output_trigger(camera_, triggers.exposure);
        } else if (!triggers.frame_start.enable &&
                   source == genicam_frame_start) {
            update_output_trigger(camera_, triggers.frame_start);
        } else if (!triggers.trigger_wait.enable &&
                   source == genicam_trigger_wait) {
            update_output_trigger(camera_, triggers.trigger_wait);
        }
    }

    last_known_settings_ = *properties;
}

void
SpinnakerCamera::get_meta(struct CameraPropertyMetadata* meta)
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
    Spinnaker::GenApi::IInteger& binning = get_binning_node(camera_);
    meta->binning = {
        .writable = IsWritable(binning),
        .low = (float)binning.GetMin(),
        .high = (float)binning.GetMax(),
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
        if (sample_type != SampleType_Unknown) {
            meta->supported_pixel_types |= (1ULL << sample_type);
        }
    }
}

void
SpinnakerCamera::query_triggering_capabilities_(CameraPropertyMetadata* meta) const
{
    // Acquire can represent at most 8 lines.
    // The first supported blackfly camera only has two (line 0 and software).
    // The first supported oryx camera has seven (lines 0-6 and software).
    // Therefore, we allow the line numbers to corresponding to their line
    // names and use line 7 for software for all cameras.
    meta->digital_lines = {
        .line_count = 8,
        .names = {
          "Line0",
          "Line1",
          "Line2",
          "Line3",
          "Line4",
          "Line5",
          "Line6",
          "Software",
        },
    };

    memset(&meta->triggers, 0, sizeof(meta->triggers));

    for (int i = 0; i < meta->digital_lines.line_count; ++i) {
        const Spinnaker::GenICam::gcstring name(meta->digital_lines.names[i]);
        // The available sources should not be dependent on the currently
        // selected trigger, so no need to change TriggerSelector.
        if (is_enum_name(camera_->TriggerSource, name)) {
            if (is_enum_name(camera_->TriggerSelector,
                             genicam_acquisition_start)) {
                meta->triggers.acquisition_start.input |= (1ULL << i);
            }
            if (is_enum_name(camera_->TriggerSelector, genicam_frame_start)) {
                meta->triggers.frame_start.input |= (1ULL << i);
            }
            if (is_enum_name(camera_->TriggerSelector,
                             genicam_exposure_active)) {
                meta->triggers.exposure.input |= (1ULL << i);
            }
        }
    }

    const Spinnaker::GenICam::gcstring original_line = *(camera_->LineSelector);
    for (int i = 0; i < meta->digital_lines.line_count; ++i) {
        const Spinnaker::GenICam::gcstring name(meta->digital_lines.names[i]);
        // Each line can have different modes and therefore different sources,
        // we iterate over the currently selected line.
        if (is_enum_name(camera_->LineSelector, name)) {
            camera_->LineSelector = name;
            if (is_enum_name(camera_->LineMode, genicam_output)) {
                if (is_enum_name(camera_->LineSource,
                                 genicam_acquisition_start)) {
                    meta->triggers.acquisition_start.output |= (1ULL << i);
                }
                if (is_enum_name(camera_->LineSource, genicam_frame_start)) {
                    meta->triggers.frame_start.output |= (1ULL << i);
                }
                if (is_enum_name(camera_->LineSource,
                                 genicam_exposure_active)) {
                    meta->triggers.exposure.output |= (1ULL << i);
                }
                // TODO: what about trigger_wait?
            }
        }
    }
    camera_->LineSelector = original_line;
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
    set_enum_node(camera_->AcquisitionMode, genicam_continuous);
    camera_->BeginAcquisition();
    started_ = true;
}

void
SpinnakerCamera::stop()
{
    const std::scoped_lock lock(lock_);
    // Disable the current trigger to prevent an effective deadlock between
    // EndAcquisition and GetNextFrame that is awaiting a trigger.
    if (IsWritable(camera_->TriggerMode)) {
        camera_->TriggerMode = genicam_off;
    }
    // Could possibly use camera_->IsStreaming instead, but the Spinnaker
    // docs are not clear enough about what that means. It's critical
    // that we call EndAcquisition exactly when needed so that associated
    // buffers are freed.
    if (started_) {
        camera_->EndAcquisition();
        started_ = false;
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
    // SDK. Cannot acquire the lock here because GetNextImage may await a
    // trigger indefinitely effectively causing a deadlock with other methods
    // that acquire the camera lock (like stop).
    Spinnaker::ImagePtr frame = camera_->GetNextImage();

    // Acquire the lock once we have the frame so that we can check if the
    // camera was stopped while we were waiting. In the case that stop was
    // called before we got here, the frame is not valid and its buffer backed
    // memory has been released, so return without explicitly releasing.
    const std::scoped_lock lock(lock_);
    if (!started_) {
        return;
    }

    if (frame->IsIncomplete()) {
        LOGE(
          "Image incomplete: (%llu) %s",
          frame->GetFrameID(),
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
              .hardware_frame_id = frame->GetFrameID(),
        };
    }

    // Possibly unneeded, but we manually release as instructed in the Spinnaker
    // Acquisition.cpp example because this image was retrieved directly from
    // the camera.
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

SpinnakerDriver::~SpinnakerDriver()
{
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
    try {
        *out = (Device*)new SpinnakerCamera(camera);
    } catch (...) {
        // If the SpinnakerCamera constructor threw, then the camera
        // may be partially initialized. Ensure it is de-initialized
        // so that the next run may succeed before re-throwing.
        if (camera->IsInitialized()) {
            LOGE("Failed to open camera, but it was initialized. "
                 "De-initializing it to clean up.");
            camera->DeInit();
        }
        throw;
    }
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
    system_->ReleaseInstance();
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
