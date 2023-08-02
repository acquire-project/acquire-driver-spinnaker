/// Calling acquire_start() twice without stopping in between should return an
/// error.

#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
#include "logger.h"

#include <cstdio>
#include <stdexcept>

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define ERR(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define CHECK(e)                                                               \
    do {                                                                       \
        if (!(e)) {                                                            \
            ERR("Expression was false:\n\t%s\n", #e);                          \
            throw std::runtime_error("Expression was false: " #e);             \
        }                                                                      \
    } while (0)
#define AOK(e) CHECK(AcquireStatus_Ok == (e))
#define DEV(e) CHECK(Device_Ok == (e))

// Expands a string argument into two arguments - str, bytes_of_string
// (includes null)
#define SIZED(str) str, sizeof(str)

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
    AcquireRuntime* runtime;
    CHECK(runtime = acquire_init(reporter));
    const DeviceManager* dm;
    CHECK(dm = acquire_device_manager(runtime));

    AcquireProperties properties = {};
    {
        AOK(acquire_get_configuration(runtime, &properties));
        DEV(device_manager_select(dm,
                                  DeviceKind_Camera,
                                  SIZED("vieworks.*") - 1,
                                  &properties.video[0].camera.identifier));
        DEV(device_manager_select(dm,
                                  DeviceKind_Storage,
                                  SIZED("Trash") - 1,
                                  &properties.video[0].storage.identifier));

        properties.video[0].camera.settings.binning = 1;
        properties.video[0].camera.settings.exposure_time_us = 1e4;
        properties.video[0].max_frame_count = 1 << 30;

        AOK(acquire_configure(runtime, &properties));
    }

    AOK(acquire_start(runtime));

    // await some data
    {
        VideoFrame *beg = nullptr, *end = nullptr;
        while (beg == end) {
            acquire_map_read(runtime, 0, &beg, &end);
            clock_sleep_ms(nullptr, 50.0);
        }
        acquire_unmap_read(runtime, 0, (uint8_t*)end - (uint8_t*)beg);
    }

    CHECK(AcquireStatus_Error == acquire_start(runtime));
    AOK(acquire_abort(runtime));

    acquire_shutdown(runtime);
    return 0;
}
