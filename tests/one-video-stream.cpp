#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
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

int
main()
{
    auto runtime = acquire_init(reporter);

    try {
        CHECK(runtime);
        auto dm = acquire_device_manager(runtime);
        CHECK(dm);

        AcquireProperties props = {};
        OK(acquire_get_configuration(runtime, &props));

        DEVOK(device_manager_select(dm,
                                    DeviceKind_Camera,
                                    SIZED(".*BFLY.*") - 1,
                                    &props.video[0].camera.identifier));
        DEVOK(device_manager_select(dm,
                                    DeviceKind_Storage,
                                    SIZED("Zarr") - 1,
                                    &props.video[0].storage.identifier));

        storage_properties_init(
          &props.video[0].storage.settings, 0, SIZED("out.zarr"), 0, 0, { .x = 1, .y = 1 });

        OK(acquire_configure(runtime, &props));

        AcquirePropertyMetadata metadata = { 0 };
        OK(acquire_get_configuration_metadata(runtime, &metadata));

        props.video[0].camera.settings.binning = 1;
        props.video[0].camera.settings.pixel_type = SampleType_u8;
        props.video[0].camera.settings.shape = {
            .x = (uint32_t)metadata.video[0].camera.shape.x.high,
            .y = (uint32_t)metadata.video[0].camera.shape.y.high,
        };
        props.video[0].camera.settings.exposure_time_us = 1e4;
        props.video[0].max_frame_count = 10;

        OK(acquire_configure(runtime, &props));

        const auto next = [](VideoFrame* cur) -> VideoFrame* {
            return (VideoFrame*)(((uint8_t*)cur) + cur->bytes_of_frame);
        };

        const auto consumed_bytes = [](const VideoFrame* const cur,
                                       const VideoFrame* const end) -> size_t {
            return (uint8_t*)end - (uint8_t*)cur;
        };

        struct clock clock;
        static double time_limit_ms = 20000.0;
        clock_init(&clock);
        clock_shift_ms(&clock, time_limit_ms);
        OK(acquire_start(runtime));
        {
            uint64_t nframes = 0;
            while (nframes < props.video[0].max_frame_count) {
                struct clock throttle;
                clock_init(&throttle);
                EXPECT(clock_cmp_now(&clock) < 0,
                       "Timeout at %f ms",
                       clock_toc_ms(&clock) + time_limit_ms);
                VideoFrame *beg, *end, *cur;
                OK(acquire_map_read(runtime, 0, &beg, &end));
                for (cur = beg; cur < end; cur = next(cur)) {
                    LOG("stream %d counting frame w id %d", 0, cur->frame_id);
                    CHECK(cur->shape.dims.width ==
                          props.video[0].camera.settings.shape.x);
                    CHECK(cur->shape.dims.height ==
                          props.video[0].camera.settings.shape.y);
                    ++nframes;
                }
                {
                    size_t n = consumed_bytes(beg, end);
                    OK(acquire_unmap_read(runtime, 0, n));
                    if (n)
                        LOG("stream %d consumed bytes %d", 0, n);
                }
                clock_sleep_ms(&throttle, 100.0f);

                LOG("stream %d nframes %d. remaining time %f s",
                    0,
                    nframes,
                    -1e-3 * clock_toc_ms(&clock));

                if (acquire_get_state(runtime) != DeviceState_Running)
                    break;
            }

            CHECK(nframes == props.video[0].max_frame_count);
        }

        OK(acquire_stop(runtime));
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
