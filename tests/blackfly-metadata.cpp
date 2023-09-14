// Checks various property setting manipulation
#include "acquire.h"

#include "device/hal/device.manager.h"
#include "device/props/camera.h"
#include "device/props/components.h"
#include "logger.h"
#include <cstdio>
#include <stdexcept>

/// Helper for passing size static strings as function args.
/// For a function: `f(char*,size_t)` use `f(SIZED("hello"))`.
/// Expands to `f("hello",5)`.
#define SIZED(str) str, sizeof(str)

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define ERR(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            char buf[1 << 8] = { 0 };                                          \
            ERR(__VA_ARGS__);                                                  \
            snprintf(buf, sizeof(buf) - 1, __VA_ARGS__);                       \
            throw std::runtime_error(buf);                                     \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false: %s", #e)
#define DEVOK(e) CHECK(Device_Ok == (e))
#define OK(e) CHECK(AcquireStatus_Ok == (e))

/// example: `ASSERT_EQ(int, "%d", 42, meaning_of_life())`
#define ASSERT_EQ(T, fmt, a, b)                                                \
    do {                                                                       \
        T a_ = (T)(a);                                                         \
        T b_ = (T)(b);                                                         \
        EXPECT(a_ == b_, "Expected %s==%s but " fmt "!=" fmt, #a, #b, a_, b_); \
    } while (0)

void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    auto stream = is_error ? stderr : stdout;
    fprintf(stream,
            "%s%s(%d) - %s: %s\n",
            is_error ? "ERROR " : "",
            file,
            line,
            function,
            msg);
    fflush(stream);
}

int
main()
{
    AcquireRuntime* runtime = 0;
    try {
        runtime = acquire_init(reporter);
        auto dm = acquire_device_manager(runtime);
        CHECK(runtime);
        CHECK(dm);

        AcquireProperties props = {};
        OK(acquire_get_configuration(runtime, &props));

        DEVOK(device_manager_select(dm,
                                    DeviceKind_Camera,
                                    SIZED(".*BFLY-U3-23S6M.*") - 1,
                                    &props.video[0].camera.identifier));
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Storage,
                                    SIZED("Trash") - 1,
                                    &props.video[0].storage.identifier));

        // Some metadata is dependent on some properties.
        // Reset the properties to something sensible.
        props.video[0].camera.settings.binning = 1;
        props.video[0].camera.settings.offset = { .x = 0, .y = 0 };
        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));

        AcquirePropertyMetadata metadata = { 0 };
        OK(acquire_get_configuration_metadata(runtime, &metadata));
        const CameraPropertyMetadata & meta = metadata.video[0].camera;
        
        // Expected values determined by inspecting blackfly metadata in spinview.
        ASSERT_EQ(uint8_t, "%d", meta.exposure_time_us.writable, 1);
        ASSERT_EQ(int, "%d", meta.exposure_time_us.type, PropertyType_FloatingPrecision);

        ASSERT_EQ(uint8_t, "%d", meta.line_interval_us.writable, 0);
        ASSERT_EQ(uint8_t, "%d", meta.readout_direction.writable, 0);

        ASSERT_EQ(uint8_t, "%d", meta.binning.writable, 1);
        ASSERT_EQ(float, "%g", meta.binning.low, 1);
        ASSERT_EQ(float, "%g", meta.binning.high, 4);
        ASSERT_EQ(int, "%d", meta.binning.type, PropertyType_FixedPrecision);
        
        ASSERT_EQ(uint8_t, "%d", meta.shape.x.writable, 1);
        ASSERT_EQ(float, "%g", meta.shape.x.low, 4);
        ASSERT_EQ(float, "%g", meta.shape.x.high, 1920);
        ASSERT_EQ(int, "%d", meta.shape.x.type, PropertyType_FixedPrecision);

        ASSERT_EQ(uint8_t, "%d", meta.shape.y.writable, 1);
        ASSERT_EQ(float, "%g", meta.shape.y.low, 2);
        ASSERT_EQ(float, "%g", meta.shape.y.high, 1200);
        ASSERT_EQ(int, "%d", meta.shape.y.type, PropertyType_FixedPrecision);
         
        ASSERT_EQ(uint8_t, "%d", meta.offset.x.writable, 1);
        ASSERT_EQ(float, "%g", meta.offset.x.low, 0);
        ASSERT_EQ(float, "%g", meta.offset.x.high, 0);
        ASSERT_EQ(int, "%d", meta.offset.x.type, PropertyType_FixedPrecision);

        ASSERT_EQ(uint8_t, "%d", meta.offset.y.writable, 1);
        ASSERT_EQ(float, "%g", meta.offset.y.low, 0);
        ASSERT_EQ(float, "%g", meta.offset.y.high, 0);
        ASSERT_EQ(int, "%d", meta.offset.y.type, PropertyType_FixedPrecision);
 
        ASSERT_EQ(unsigned int, "0x%x", (unsigned int)meta.supported_pixel_types, (1U << SampleType_u8) | (1U << SampleType_u16));
 
        ASSERT_EQ(uint8_t, "0x%x", meta.triggers.acquisition_start.input, 0);
        ASSERT_EQ(uint8_t, "0x%x", meta.triggers.acquisition_start.output, 0);
        ASSERT_EQ(uint8_t, "0x%x", meta.triggers.exposure.input, 0b1000'0001);
        ASSERT_EQ(uint8_t, "0x%x", meta.triggers.exposure.output, 0b0000'0010);
        ASSERT_EQ(uint8_t, "0x%x", meta.triggers.frame_start.input, 0b1000'0001);
        ASSERT_EQ(uint8_t, "0x%x", meta.triggers.frame_start.output, 0);

        OK(acquire_shutdown(runtime));
        LOG("OK");
        return 0;
    } catch (const std::runtime_error& e) {
        ERR("Runtime error: %s", e.what());
    } catch (...) {
        ERR("Uncaught exception");
    }
    acquire_shutdown(runtime);
    return 1;
}