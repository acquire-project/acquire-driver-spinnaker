/// @file Driver wrapping the spinnaker SDK.
/// Written to target the ORX-10G-51S5
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

constexpr size_t NBUFFERS = 16;

#define countof(e) (sizeof(e) / sizeof(*(e)))

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

// #define echo(str) echo_(str, __LINE__)
#define echo(str) str

namespace {

std::string
echo_(const std::string& val, int line)
{
    LOG("ECHO STRING %s (line %d)", val.c_str(), line);
    return val;
}

struct SpinnakerCamera final : private Camera
{
    explicit SpinnakerCamera(Spinnaker::CameraPtr& camera);
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
    mutable Spinnaker::CameraPtr pCam_;
    struct CameraProperties last_known_settings_;
    struct CameraPropertyMetadata last_known_capabilities_;
    uint64_t frame_id_;
    mutable std::mutex lock_;

    // Maps GenICam PixelFormat names to SampleType.
    const std::unordered_map<Spinnaker::PixelFormatEnums, SampleType>
      px_type_table_;
    const std::unordered_map<SampleType, Spinnaker::PixelFormatEnums>
      px_type_inv_table_;

    // Maps GenICam TriggerActivation names to TriggerEdge
    const std::unordered_map<std::string, TriggerEdge> trig_edge_table_;
    const std::unordered_map<TriggerEdge, std::string> trig_edge_inv_table_;

    enum TrigSrc
    {
        Trig_Line0 = 0,
        Trig_Software = 1,
        Trig_Unknown
    };
    // Maps GenICam TriggerSource to TrigSrc
    const std::unordered_map<std::string, TrigSrc> trig_src_table_;

    void query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const;
    void query_binning_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_offset_capabilities_(CameraPropertyMetadata* meta) const;
    void query_roi_shape_capabilities_(CameraPropertyMetadata* meta) const;
    void query_pixel_type_capabilities_(CameraPropertyMetadata* meta) const;
    static void query_triggering_capabilities_(CameraPropertyMetadata* meta);

    float maybe_set_exposure_time_us_(float target_us, float last_value_us);
    uint8_t maybe_set_binning(uint8_t target, uint8_t last_value);
    SampleType maybe_set_px_type(SampleType target, SampleType last_known);
    CameraProperties::camera_properties_offset_s maybe_set_offset(
      CameraProperties::camera_properties_offset_s target,
      CameraProperties::camera_properties_offset_s last);
    CameraProperties::camera_properties_shape_s maybe_set_shape(
      CameraProperties::camera_properties_shape_s target,
      CameraProperties::camera_properties_shape_s last);
    void maybe_set_trigger(Trigger& target, const Trigger& last);
};

struct SpinnakerDriver final : public Driver
{
    SpinnakerDriver();

    uint32_t device_count();
    void describe(DeviceIdentifier* identifier, uint64_t i);
    void open(uint64_t device_id, struct Device** out);
    static void close(struct Device* in);
};

template<typename K, typename V>
V
at_or(const std::unordered_map<K, V>& table, const K& key, V dflt)
{
    const auto it = table.find(key);
    if (it == std::end(table)) {
        return dflt;
    } else {
        return it->second;
    }
}

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

SpinnakerCamera::SpinnakerCamera(Spinnaker::CameraPtr & pCam)
  : Camera{ .set = ::spinnakercam_set,
            .get = ::spinnakercam_get,
            .get_meta = ::spinnakercam_get_meta,
            .get_shape = ::spinnakercam_get_shape,
            .start = ::spinnakercam_start,
            .stop = ::spinnakercam_stop,
            .execute_trigger = ::spinnakercam_execute_trigger,
            .get_frame = ::spinnakercam_get_frame,
  }
  , pCam_(pCam)
  , last_known_settings_{}
  , px_type_table_ {
        { Spinnaker::PixelFormatEnums::PixelFormat_Mono8, SampleType_u8 },
        { Spinnaker::PixelFormatEnums::PixelFormat_Mono10, SampleType_u10 },
        { Spinnaker::PixelFormatEnums::PixelFormat_Mono12, SampleType_u12 },
        { Spinnaker::PixelFormatEnums::PixelFormat_Mono14, SampleType_u14 },
        { Spinnaker::PixelFormatEnums::PixelFormat_Mono16, SampleType_u16 },
    }
  , px_type_inv_table_ {
      { SampleType_u8 , Spinnaker::PixelFormatEnums::PixelFormat_Mono8 },
      { SampleType_u10, Spinnaker::PixelFormatEnums::PixelFormat_Mono10},
      { SampleType_u12, Spinnaker::PixelFormatEnums::PixelFormat_Mono12},
      { SampleType_u14, Spinnaker::PixelFormatEnums::PixelFormat_Mono14},
      { SampleType_u16, Spinnaker::PixelFormatEnums::PixelFormat_Mono16},
  }
  ,trig_edge_table_{
      { "RisingEdge", TriggerEdge_Rising },
      { "FallingEdge", TriggerEdge_Falling },
      { "AnyEdge", TriggerEdge_AnyEdge },
      { "LevelHigh", TriggerEdge_LevelHigh },
      { "LevelLow", TriggerEdge_LevelLow },
  }
  ,trig_edge_inv_table_{
      { TriggerEdge_Rising, "RisingEdge"},
      { TriggerEdge_Falling, "FallingEdge"},
      { TriggerEdge_AnyEdge, "AnyEdge"},
      { TriggerEdge_LevelHigh, "LevelHigh"},
      { TriggerEdge_LevelLow, "LevelLow"},
  }
  , trig_src_table_{
      { "Line0", Trig_Line0},
      { "Software", Trig_Software},
  }
  , frame_id_(0)
{
    // TODO: may need equivalent
    // grabber_.stop(); // just in case
    // grabber_.execute<ES::RemoteModule>("AcquisitionStop");
    // grabber_.setString<ES::RemoteModule>("TriggerMode", "Off");
    get(&last_known_settings_);
    get_meta(&last_known_capabilities_);
}

SpinnakerCamera::~SpinnakerCamera()
{
    try {
        stop();
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

    last_known_settings_.pixel_type = maybe_set_px_type(
      properties->pixel_type, last_known_settings_.pixel_type);

    last_known_settings_.offset =
      maybe_set_offset(properties->offset, last_known_settings_.offset);

    last_known_settings_.shape =
      maybe_set_shape(properties->shape, last_known_settings_.shape);

    maybe_set_trigger(properties->input_triggers.frame_start,
                      last_known_settings_.input_triggers.frame_start);

    // TODO
    // grabber_.reallocBuffers(NBUFFERS);
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

float
SpinnakerCamera::maybe_set_exposure_time_us_(float target_us,
                                             float last_value_us)
{
    return last_value_us;
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
    // TODO
    meta->exposure_time_us = {
        .writable = false,
        .low = 10.0,
        .high = 100000.0,
        .type = PropertyType_FloatingPrecision,
    };
}

void
SpinnakerCamera::query_binning_capabilities_(CameraPropertyMetadata* meta) const
{
    // TODO
    meta->binning = {
        .writable = false,
        .low = 1.0,
        .high = 4.0,
        .type = PropertyType_FixedPrecision,
    };
}
void
SpinnakerCamera::query_roi_offset_capabilities_(
  CameraPropertyMetadata* meta) const
{
    // TODO
    meta->offset = {
        .x = {
          .writable = false,
          .low = 0.0,
          .high = 0.0,
          .type = PropertyType_FixedPrecision,
        },
        .y = {
          .writable = false,
          .low = 0.0,
          .high = 0.0,
          .type = PropertyType_FixedPrecision,
        },
    };
}
void
SpinnakerCamera::query_roi_shape_capabilities_(
  CameraPropertyMetadata* meta) const
{
    // TODO
    meta->shape = {
        .x = {
          .writable = false,
          .low = (float)pCam_->Width.GetValue(),
          .high = (float)pCam_->Width.GetValue(),
          .type = PropertyType_FixedPrecision,
        },
        .y = {
          .writable = false,
          .low = (float)pCam_->Height.GetValue(),
          .high = (float)pCam_->Height.GetValue(),
          .type = PropertyType_FixedPrecision,
        },
    };
}

void
SpinnakerCamera::query_pixel_type_capabilities_(
  CameraPropertyMetadata* meta) const
{
    // TODO
    meta->supported_pixel_types = 0;
    for (const auto& entry : px_type_table_) {
        meta->supported_pixel_types |= (1ULL << entry.first);
    }
}

void
SpinnakerCamera::query_triggering_capabilities_(CameraPropertyMetadata* meta)
{
}

void
SpinnakerCamera::get(struct CameraProperties* properties)
{
    const std::scoped_lock lock(lock_);
    *properties = {
        .exposure_time_us = (float)pCam_->ExposureTime.GetValue(),
        .binning = (uint8_t)pCam_->BinningHorizontal.GetValue(),
        .pixel_type =
          at_or(px_type_table_, pCam_->PixelFormat.GetValue(), SampleType_Unknown),
        .offset = {
          .x = (uint32_t)pCam_->OffsetX.GetValue(),
          .y = (uint32_t)pCam_->OffsetY.GetValue(),
        },
        .shape = {
          .x = (uint32_t)pCam_->Width.GetValue(),
          .y = (uint32_t)pCam_->Height.GetValue(),
        },
    };
    last_known_settings_ = *properties;
}
uint8_t
SpinnakerCamera::maybe_set_binning(uint8_t target, uint8_t last_value)
{
    return last_value;
}

SampleType
SpinnakerCamera::maybe_set_px_type(SampleType target, SampleType last_known)
{
    CHECK(target < SampleTypeCount);
    return last_known;
}
CameraProperties::camera_properties_offset_s
SpinnakerCamera::maybe_set_offset(
  CameraProperties::camera_properties_offset_s target,
  CameraProperties::camera_properties_offset_s last)
{
    return last;
}

CameraProperties::camera_properties_shape_s
SpinnakerCamera::maybe_set_shape(
  CameraProperties::camera_properties_shape_s target,
  CameraProperties::camera_properties_shape_s last)
{
    return last;
}

void
SpinnakerCamera::maybe_set_trigger(Trigger& target, const Trigger& last)
{
}

void
SpinnakerCamera::start()
{
    const std::scoped_lock lock(lock_);
    frame_id_ = 0;
    pCam_->Init();

    Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam_->GetTLDeviceNodeMap();
    Spinnaker::GenApi::INodeMap& nodeMap = pCam_->GetNodeMap();

    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode =
      nodeMap.GetNode("AcquisitionMode");
    if (!IsReadable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
        LOGE("Unable to set acquisition mode to continuous (enum retrieval). "
             "Aborting...\n");
        return;
    }

    // Retrieve entry node from enumeration node
    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous =
      ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsReadable(ptrAcquisitionModeContinuous)) {
        LOGE("Unable to get or set acquisition mode to continuous (entry "
             "retrieval). Aborting...\n");
        return;
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous =
      ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    LOG("Acquisition mode set to continuous...\n");

    // Begin acquiring images
    //
    // *** NOTES ***
    // What happens when the camera begins acquiring images depends on the
    // acquisition mode. Single frame captures only a single image, multi
    // frame captures a set number of images, and continuous captures a
    // continuous stream of images. Because the example calls for the
    // retrieval of 10 images, continuous mode has been set.
    //
    // *** LATER ***
    // Image acquisition must be ended when no more images are needed.
    pCam_->BeginAcquisition();

    LOG("Acquiring images...\n");
}

void
SpinnakerCamera::stop()
{
    const std::scoped_lock lock(lock_);
    pCam_->DeInit();
}

void
SpinnakerCamera::get_shape(struct ImageShape* shape) const
{
    const std::scoped_lock lock(lock_);

    const Spinnaker::Camera* cam = pCam_.get();

    uint32_t w = cam->Width.GetValue();
    uint32_t h = cam->Height.GetValue();
    *shape = {
        .dims = {
            .channels = 1,
            .width = w,
            .height = h,
            .planes = 1,
        },
        .strides = {
          .channels = 1,
          .width = 1,
          .height = w,
          .planes = w*h,
        },
        .type = at_or(px_type_table_, cam->PixelFormat.GetValue(), SampleType_Unknown),
    };
}
void
SpinnakerCamera::execute_trigger() const
{
    const std::scoped_lock lock(lock_);
    // TODO
    // grabber_.execute<ES::RemoteModule>("TriggerSoftware");
}

void
SpinnakerCamera::get_frame(void* im, size_t* nbytes, struct ImageInfo* info)
{
    // TODO: check if similar.
    // Locking: This function is basically read-only when it comes to EGCamera
    // state so it doesn't need a scoped lock.

    // TODO: Check if we should pass explicit timeout.
    Spinnaker::ImagePtr pResultImage = pCam_->GetNextImage();

    if (pResultImage->IsIncomplete()) {
        // Retrieve and print the image status description
        LOGE("Image incomplete: %s\n",
             Spinnaker::Image::GetImageStatusDescription(
               pResultImage->GetImageStatus()));
    } else {
        // Print image information; height and width recorded in pixels
        //
        // *** NOTES ***
        // Images have quite a bit of available metadata including
        // things such as CRC, image status, and offset values, to
        // name a few.
        const size_t width = pResultImage->GetWidth();
        const size_t height = pResultImage->GetHeight();

        LOG("Grabbed width = %d, height = %d", width, height);

        const Spinnaker::IImage* frame = pResultImage.get();

        // TODO: check resolution of this timestamp
        const auto timestamp_ns = frame->GetTimeStamp();

        CHECK(*nbytes >= frame->GetBufferSize());
        EXPECT(frame->GetData(), "Expected non-null pointer");

        std::memcpy(im, frame->GetData(), frame->GetBufferSize());
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
                  .type = at_or(px_type_table_, frame->GetPixelFormat(), SampleType_Unknown),
              },
              .hardware_timestamp = timestamp_ns,
              .hardware_frame_id = frame_id_++,
        };
    }

    pResultImage->Release();
}

//
//      SPINNAKERDRIVER IMPLEMENTATION
//

SpinnakerDriver::SpinnakerDriver()
  : Driver{
      .device_count = ::spinnakercam_device_count,
      .describe = ::spinnakercam_describe,
      .open = ::spinnakercam_open,
      .close = ::spinnakercam_close,
      .shutdown = ::spinnakercam_shutdown_,
  }
{
}

void
SpinnakerDriver::describe(DeviceIdentifier* identifier, uint64_t i)
{
    // DeviceManager device_id expects a uint8
    EXPECT(i < (1 << 8), "Expected a uint8 device index. Got: %llu", i);

    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraList camList = system->GetCameras();
    Spinnaker::CameraPtr pCam = camList.GetByIndex(i);
    Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetTLDeviceNodeMap();

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
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraList camList = system->GetCameras();
    return camList.GetSize();
}

void
SpinnakerDriver::open(uint64_t device_id, struct Device** out)
{
    CHECK(out);
    EXPECT(device_id < (1ULL << 8 * sizeof(int)) - 1,
           "Expected an int32 device id. Got: %llu",
           device_id);
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraList camList = system->GetCameras();
    Spinnaker::CameraPtr pCam = camList.GetByIndex(device_id);
    *out = (Device*)new SpinnakerCamera(pCam);
}

void
SpinnakerDriver::close(struct Device* in)
{
    CHECK(in);
    auto camera = (SpinnakerCamera*)in;
    delete camera;
}

} // end anonymous namespace

acquire_export struct Driver*
acquire_driver_init_v0(acquire_reporter_t reporter)
{
    try {
        logger_set_reporter(reporter);
        return new SpinnakerDriver;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return nullptr;
}
