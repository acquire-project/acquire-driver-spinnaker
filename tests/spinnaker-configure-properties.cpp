// Checks various property setting manipulation
#include "acquire.h"
#include "device/hal/device.manager.h"
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

        // After this tests passes, the camera should have reasonable
        // default property values.

        // Binning, shape, and offset are coupled since certain combinations do
        // not define valid regions on the sensor. The following tests cover
        // some important cases, but not everything.
        
        // First set the region to be the whole sensor at native resolution.
        props.video[0].camera.settings.binning = 1;
        props.video[0].camera.settings.offset = { .x = 0, .y = 0 };
        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 1);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1920);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1200);
       
        // Trying to set the offset should have no effect because the current
        // shape prevents it.
        props.video[0].camera.settings.offset = { .x = 120, .y = 100 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);

        // But we can shrink the shape and change the offset together.
        props.video[0].camera.settings.offset = { .x = 120, .y = 100};
        props.video[0].camera.settings.shape = { .x = 960, .y = 600 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 960);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 600);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 120);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 100);

        // Now trying to set the shape should have no effect because the current
        // offset prevents it.
        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 960);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 600);

        // So change the shape and offset together.
        props.video[0].camera.settings.offset = { .x = 0, .y = 0 };
        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1920);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1200);

        // Trying to set binning while the shape is too large should also do nothing.
        props.video[0].camera.settings.binning = 2;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 1);

        // So change binning and shape together.
        props.video[0].camera.settings.binning = 2;
        props.video[0].camera.settings.shape = { .x = 960, .y = 600 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 2);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 960);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 600);

        // Reset the region back to the full sensor at native resolution
        // before changing other properties.
        props.video[0].camera.settings.binning = 1;
        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 1);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1920);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1200);

        props.video[0].camera.settings.pixel_type = SampleType_u16;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(SampleType,
                  "%d",
                  props.video[0].camera.settings.pixel_type,
                  SampleType_u16);
        props.video[0].camera.settings.pixel_type = SampleType_u8;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(SampleType,
                  "%d",
                  props.video[0].camera.settings.pixel_type,
                  SampleType_u8);

        // Exposure time is tricky because only certain values are supported.
        // Here we set to the min and max values because those should be supported
        // and can also be compared with equality.
        AcquirePropertyMetadata metadata = { 0 };
        OK(acquire_get_configuration_metadata(runtime, &metadata));

        props.video[0].camera.settings.exposure_time_us = metadata.video[0].camera.exposure_time_us.low;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(float, "%f",
        props.video[0].camera.settings.exposure_time_us, metadata.video[0].camera.exposure_time_us.low);
        props.video[0].camera.settings.exposure_time_us = metadata.video[0].camera.exposure_time_us.high;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(float, "%f",
        props.video[0].camera.settings.exposure_time_us, metadata.video[0].camera.exposure_time_us.high);

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