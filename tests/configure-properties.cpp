// Checks various property setting manipulation
#include "acquire.h"
#include "device/hal/device.manager.h"
#include "logger.h"
#include <cstdio>
#include <stdexcept>

void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    fprintf(is_error ? stderr : stdout,
            "%s%s(%d) - %s: %s\n",
            is_error ? "ERROR " : "",
            file,
            line,
            function,
            msg);
}

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

        OK(acquire_configure(runtime, &props));

        AcquirePropertyMetadata metadata = { 0 };
        OK(acquire_get_configuration_metadata(runtime, &metadata));

        // After this tests passes, the camera should have reasonable
        // default property values.

        // TODO: exposure time is tricky :(
        // props.video[0].camera.settings.exposure_time_us = 200;
        // OK(acquire_configure(runtime, &props));
        // ASSERT_EQ(float, "%f",
        // props.video[0].camera.settings.exposure_time_us, 200);
        // props.video[0].camera.settings.exposure_time_us = 100;
        // OK(acquire_configure(runtime, &props));
        // ASSERT_EQ(float, "%f",
        // props.video[0].camera.settings.exposure_time_us, 100);

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

        // Shape and offset are coupled since certain combinations are not valid
        // given the fixed sensor size.
        props.video[0].camera.settings.shape = { .x = 32, .y = 16 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 32);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 16);

        props.video[0].camera.settings.offset = { .x = 64, .y = 42 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 64);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 42);

        props.video[0].camera.settings.offset = { .x = 0, .y = 0 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.x, 0);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.offset.y, 0);

        props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.x, 1920);
        ASSERT_EQ(uint32_t, "%d", props.video[0].camera.settings.shape.y, 1200);

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
