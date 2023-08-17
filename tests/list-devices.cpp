/// @file
/// @brief Lists the devices exposed by this driver.
/// Exercises the device enumeration interface.

#include "platform.h"
#include "logger.h"
#include "device/kit/driver.h"

#include <cstdio>

#define L aq_logger
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define LOGE(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            LOGE(__VA_ARGS__);                                                 \
            goto Error;                                                        \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false:\n\t%s", #e)

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

typedef struct Driver* (*init_func_t)(void (*reporter)(int is_error,
                                                       const char* file,
                                                       int line,
                                                       const char* function,
                                                       const char* msg));

int
main()
{
    logger_set_reporter(reporter);
    lib lib{};
    CHECK(lib_open_by_name(&lib, "acquire-driver-spinnaker"));
    {
        auto init = (init_func_t)lib_load(&lib, "acquire_driver_init_v0");
        auto driver = init(reporter);
        CHECK(driver);
        const auto n = driver->device_count(driver);
        LOGE("Device count: %d", n);
        for (uint32_t i = 0; i < n; ++i) {
            DeviceIdentifier id{};
            char buf[1 << 7] = { 0 };
            CHECK(driver->describe(driver, &id, i) == Device_Ok);
            device_identifier_as_debug_string(buf, sizeof(buf), &id);
            LOG("%d %s", i, buf);
        }
        driver->shutdown(driver);
    }
    LOG("OK");
    lib_close(&lib);
    return 0;
Error:
    lib_close(&lib);
    return 1;
}
