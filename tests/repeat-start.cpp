#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
#include "logger.h"

#include <cstdio>
#include <exception>
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

static void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    printf("%s%s(%d) - %s: %s\n",
           is_error ? "ERROR " : "",
           file,
           line,
           function,
           msg);
}

int
main()
{
    AcquireRuntime* runtime = 0;
    try {
        CHECK(runtime = acquire_init(reporter));
        const DeviceManager* dm;
        CHECK(dm = acquire_device_manager(runtime));

        AcquireProperties properties = {};
        OK(acquire_get_configuration(runtime, &properties));

        DEVOK(device_manager_select(dm,
                                    DeviceKind_Camera,
                                    SIZED(".*BFLY.*") - 1,
                                    &properties.video[0].camera.identifier));
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Storage,
                                    SIZED("Trash") - 1,
                                    &properties.video[0].storage.identifier));

        properties.video[0].max_frame_count = 10;
        OK(acquire_configure(runtime, &properties));

        for (auto i = 0; i < 10; ++i) {
            struct clock clock = {};
            clock_init(&clock);
            OK(acquire_start(runtime));
            OK(acquire_stop(runtime));
            LOG("Start/Stop cycle took %f ms", clock_toc_ms(&clock));
        }
        OK(acquire_shutdown(runtime));
        LOG("OK");
        return 0;
    } catch (const std::exception& e) {
        ERR("Exception: %s", e.what());
    } catch (...) {
        ERR("Exception: (unknown)");
    }
    acquire_shutdown(runtime);
    return 1;
}
