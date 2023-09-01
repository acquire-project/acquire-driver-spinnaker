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
                                    SIZED(".*BFLY.*") - 1,
                                    &props.video[0].camera.identifier));
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Storage,
                                    SIZED("Trash") - 1,
                                    &props.video[0].storage.identifier));

        // After this tests passes, the camera should have reasonable
        // default property values.

        // Shape and offset are coupled since certain combinations are not valid
        // given the fixed sensor size, so set a small shape first, followed by
        // a small offset, then back to the full sensor.
        
        // First set the region to be the whole sensor.
        props.video[0].camera.settings.binning = 1;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 1);

        props.video[0].camera.settings.offset = { .x = 0, .y = 0 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);

        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1920);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1200);
       
        // Trying to set the offset should have no effect because the current
        // shape prevents it.
        props.video[0].camera.settings.offset = { .x = 120, .y = 100 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);

        // But we can shrink the shape.
        props.video[0].camera.settings.shape = { .x = 960, .y = 600 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 960);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 600);

        // And now we can change the offset.
        props.video[0].camera.settings.offset = { .x = 120, .y = 100};
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 120);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 100);

        // Now trying to set the shape back to the full field of view should
        // partially work, but will be clamped according to the current offset.
        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1800);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1100);

        // Reset everything back to the full sensor for testing other settings.
        props.video[0].camera.settings.offset = { .x = 0, .y = 0 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);

        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1920);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1200);

        props.video[0].camera.settings.binning = 2;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 2);
        props.video[0].camera.settings.binning = 1;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.binning, 1);

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
