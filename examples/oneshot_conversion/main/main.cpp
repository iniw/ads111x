#include <ads111x/ads111x.hpp>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" void app_main() {
    i2c_master_bus_handle_t bus_handle;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_4,
        .scl_io_num = GPIO_NUM_5,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    auto device_handle = ads111x::init(bus_handle, ads111x::AddrSelection::GND, 400'000).value();

    using namespace ads111x::reg;

    while (true) {
        ads111x::write(
            device_handle,
            Config {
                .os = Config::OS::StartSingleConversion,
            })
            .value();

        // Wait a bit for the conversion to finish
        vTaskDelay(pdMS_TO_TICKS(500));

        ads111x::write(
            device_handle,
            AddressPointer {
                .p = AddressPointer::Register::Conversion,
            })
            .value();

        auto conversion = ads111x::read<Conversion>(device_handle).value();
        ESP_LOGI("Example", "Conversion result = %d", conversion.d);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
