#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
#include "logger.h"

#include <cstdio>
#include <exception>
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

// TODO: (nclack) add support for some default reporters, esp for tests
//          aq_logger is sort of going down that road, but just
//          an awkward api design.  Why that and reporter?
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
        {
            AOK(acquire_get_configuration(runtime, &properties));
            DEV(device_manager_select(dm,
                                      DeviceKind_Camera,
                                      "VIEWORKS.*",
                                      10,
                                      &properties.video[0].camera.identifier));
            DEV(device_manager_select(dm,
                                      DeviceKind_Storage,
                                      "Trash",
                                      5,
                                      &properties.video[0].storage.identifier));

            // FIXME: (nclack) there's not way to get default properties for the
            //        properties that depend on device selection.
            //        `acquire_configure` fails with binning=0 bc of reasons on
            //        set. Users should be able to query for a valid default.
            //        The can't _just_ set the camera and then query.
            //        This is a result of an api choice I value: bulk and
            //        stateless property updates. But then we need something
            //        else for device dependant defaults.
            properties.video[0].camera.settings.binning = 1;
            properties.video[0].max_frame_count = 10;

            AOK(acquire_configure(runtime, &properties));
        }

        for (auto i = 0; i < 10; ++i) {
            struct clock clock = {};
            clock_init(&clock);
            AOK(acquire_start(runtime));
            AOK(acquire_stop(runtime));
            LOG("Start/Stop cycle took %f ms", clock_toc_ms(&clock));
        }
        acquire_shutdown(runtime);
        return 0;
    } catch (const std::exception& e) {
        ERR("Exception: %s", e.what());
    } catch (...) {
        ERR("Exception: (unknown)");
    }
    acquire_shutdown(runtime);
    return 1;
}
