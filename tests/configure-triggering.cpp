// Checks various trigger setting manipulation
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

        // Enable frame start input trigger on line 0.
        props.video[0].camera.settings.input_triggers.frame_start.line = 0;
        props.video[0].camera.settings.input_triggers.frame_start.enable = 1;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.input_triggers.frame_start.line, 0);
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.input_triggers.frame_start.enable, 1);

        // Enable frame start input trigger on as a software trigger.
        props.video[0].camera.settings.input_triggers.frame_start.line = 2;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.input_triggers.frame_start.line, 2);
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.input_triggers.frame_start.enable, 1);

        // Disable frame start input trigger on line 0.
        props.video[0].camera.settings.input_triggers.frame_start.line = 0;
        props.video[0].camera.settings.input_triggers.frame_start.enable = 0;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.input_triggers.frame_start.line, 0);
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.input_triggers.frame_start.enable, 0);

        // Enable exposure output trigger on line 1.
        props.video[0].camera.settings.output_triggers.exposure.line = 1;
        props.video[0].camera.settings.output_triggers.exposure.enable = 1;
        OK(acquire_configure(runtime, &props));
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.output_triggers.exposure.line, 1);
        ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.output_triggers.exposure.enable, 1);

        // TODO: enable when disabling exposure is supported.
        // Disable exposure output trigger on line 1.
        //props.video[0].camera.settings.output_triggers.exposure.enable = 0;
        //OK(acquire_configure(runtime, &props));
        //ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.output_triggers.exposure.line, 1);
        //ASSERT_EQ(uint8_t, "%d", props.video[0].camera.settings.output_triggers.exposure.enable, 0);

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
