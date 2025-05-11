#include <bit>
#include <cstring>

#include "ads111x.hpp"

namespace ads111x {

static std::expected<void, esp_err_t> write_raw(i2c_master_dev_handle_t handle, reg::AddressPointer address_pointer) {
    if (auto error = i2c_master_transmit(handle, reinterpret_cast<const uint8_t*>(&address_pointer), sizeof(reg::AddressPointer), CONFIG_ADS1115_I2C_TIMEOUT))
        return std::unexpected(error);

    return {};
}

static std::expected<void, esp_err_t> write_raw(i2c_master_dev_handle_t handle, reg::AddressPointer address_pointer, uint16_t data) {
    uint8_t wire_buffer[sizeof(reg::AddressPointer) + sizeof(uint16_t)];

    uint16_t big_endian_data = std::byteswap(data);

    std::memcpy(wire_buffer, &address_pointer, sizeof(reg::AddressPointer));
    std::memcpy(wire_buffer + sizeof(reg::AddressPointer), &big_endian_data, sizeof(big_endian_data));

    if (auto error = i2c_master_transmit(handle, wire_buffer, sizeof(wire_buffer), CONFIG_ADS1115_I2C_TIMEOUT))
        return std::unexpected(error);

    return {};
}

static std::expected<uint16_t, esp_err_t> read_raw(i2c_master_dev_handle_t handle) {
    uint16_t big_endian_data;

    if (auto error = i2c_master_receive(handle, reinterpret_cast<uint8_t*>(&big_endian_data), sizeof(big_endian_data), CONFIG_ADS1115_I2C_TIMEOUT))
        return std::unexpected(error);

    return std::byteswap(big_endian_data);
}

std::expected<i2c_master_dev_handle_t, esp_err_t> init(i2c_master_bus_handle_t bus_handle, AddrSelection addr_line, uint32_t scl_frequency) {
    i2c_master_dev_handle_t device_handle;

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = static_cast<uint16_t>(addr_line),
        .scl_speed_hz = scl_frequency,
        .scl_wait_us = 0,
        .flags {
            .disable_ack_check = false,
        },
    };

    if (auto error = i2c_master_bus_add_device(bus_handle, &device_config, &device_handle))
        return std::unexpected(error);

    return device_handle;
}

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t device_handle, reg::AddressPointer address_pointer) {
    return write_raw(device_handle, address_pointer);
}

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t device_handle, reg::Config config) {
    return write_raw(
        device_handle,
        reg::AddressPointer {
            .p = reg::AddressPointer::Register::Config,
        },
        std::bit_cast<uint16_t>(config));
}

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t device_handle, reg::Lo_thresh lo_thresh) {
    return write_raw(
        device_handle,
        reg::AddressPointer {
            .p = reg::AddressPointer::Register::Lo_thresh,
        },
        std::bit_cast<uint16_t>(lo_thresh));
}

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t device_handle, reg::Hi_thresh hi_thresh) {
    return write_raw(
        device_handle,
        reg::AddressPointer {
            .p = reg::AddressPointer::Register::Hi_thresh,
        },
        std::bit_cast<uint16_t>(hi_thresh));
}

template<>
std::expected<reg::Conversion, esp_err_t> read<reg::Conversion>(i2c_master_dev_handle_t device_handle) {
    return read_raw(device_handle).transform([](uint16_t raw) { return std::bit_cast<reg::Conversion>(raw); });
}

template<>
std::expected<reg::Config, esp_err_t> read<reg::Config>(i2c_master_dev_handle_t device_handle) {
    return read_raw(device_handle).transform([](uint16_t raw) { return std::bit_cast<reg::Config>(raw); });
}

template<>
std::expected<reg::Lo_thresh, esp_err_t> read<reg::Lo_thresh>(i2c_master_dev_handle_t device_handle) {
    return read_raw(device_handle).transform([](uint16_t raw) { return std::bit_cast<reg::Lo_thresh>(raw); });
}

template<>
std::expected<reg::Hi_thresh, esp_err_t> read<reg::Hi_thresh>(i2c_master_dev_handle_t device_handle) {
    return read_raw(device_handle).transform([](uint16_t raw) { return std::bit_cast<reg::Hi_thresh>(raw); });
}

}
