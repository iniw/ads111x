# Installation
Just place this repository in your project's `components/` directory and then add it as a requirement of your `main` component:

```cmake
# main/CMakeLists.txt

idf_component_register(
    SRCS
        main.cpp
        # ...
    PRIV_REQUIRES
        ads111x
        # ...
)
```

The easiest way to do that is to just clone the repository directly to that folder:
```sh
# (from your project's root directory)
$ git clone https://github.com/iniw/ads111x.git components/
```

You could also add it as a git submodule, which makes it easier to update:
```sh
# (from your project's root directory)
$ git submodule add https://github.com/iniw/ads111x.git components/
```

You can also consume it through CMake's FetchContent module, which allows pinning to a specific release tag, see [this example](./examples/cmake_fetch_content/CMakeLists.txt).

# Usage
The library provides only three functions:
- `init`: Initializes the I2C peripheral
- `write`: Writes to a register
- `read`: Reads from a register

Basic example:
```cpp
i2c_master_bus_handle_t bus_handle = /* The handle of your I2C master */;

constexpr auto SCL_FREQUENCY = 400'000u;                     // The I2C transfer's clock speed for the peripheral
constexpr auto ADDR_SELECTION = ads111x::AddrSelection::GND; // What your peripheral's ADDR pin is wired to

std::expected<i2c_master_dev_handle_t, esp_err_t> handle_or_error = ads111x::init(bus_handle, ADDR_SELECTION, SCL_FREQUENCY);
if (!handle_or_error.has_value()) {
    // Handle error
}

// Bring all registers (Config, AddressPointer, Conversion, ...) into scope
using namespace ads111x::reg;

// Configure the device
std::expected<void, esp_err_t> write_or_error = ads111x::write(
    *handle_or_error,
    Config {
        // Read 250 samples per second
        .dr = Config::DataRate::_250SPS,
        // Continuously generate new samples
        .mode = Config::Mode::ContinuousConversion,
        // Read from 0.0V to 4.096V
        .pga = Config::PGA::FSR_4_096V,
        // Use the A0 pin as the positive reference (AINP) and GND as the negative reference (AINN)
        .mux = Config::Mux::AINP_AIN0_AINN_GND,
    });

if (!write_or_error.has_value()) {
    // Handle error
}

// Point the address pointer to the conversion register so that we can read from it
write_or_error = ads111x::write(
    *handle_or_error,
    AddressPointer {
        .p = AddressPointer::Register::Conversion,
    });

if (!write_or_error.has_value()) {
    // Handle error
}

std::expected<Conversion, esp_err_t> conversion_or_error = ads111x::read<Conversion>(*handle_or_error);
if (!conversion_or_error.has_value()) {
    // Handle error
}

// We set the PGA to gives us a full-scale-range of 4.096V, so our digital reading will be relative to that scale.
float a0_voltage = 4.096f * (static_cast<float>(conversion_or_error->d) / INT16_MAX);
ESP_LOGI("Example", "Voltage in A0 = %f", a0_voltage);
```

See also the [examples](./examples) folder.

# Design and safety notes
The library is purposefully bare-bones and unopinionated. It is meant to provide low-level (but expressive) access to the registers and nothing else.

The library uses overload sets (in the case of `write`) and [explicit template specialization](https://en.cppreference.com/w/cpp/language/template_specialization) (in the case of `read`), making the API fully type-safe:

```cpp
// error: no matching function for call to 'write(i2c_master_dev_t*, int)'
ads111x::write(device_handle, 0);

// error: static assertion failed: Invalid register for `read()`
ads111x::read<int>(device_handle);
```

The registers are defined using bitfields and are extensively documented with descriptions, names and comments straight from the [datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf):
```cpp
/// Address Pointer Register (address = N/A) [reset = N/A]
///
/// All four registers are accessed by writing to the Address Pointer register
struct [[gnu::packed]] AddressPointer {
    /// Register address pointer
    enum class Register : unsigned {
        /// Conversion register
        Conversion = 0b00,
        /// Config register
        Config = 0b01,
        /// Lo_thresh register
        Lo_thresh = 0b10,
        /// Hi_thresh register
        Hi_thresh = 0b11,
    } p : 2
        = Register::Conversion;

    unsigned reserved : 6 = 0b000000;
};

static_assert(sizeof(AddressPointer) == sizeof(uint8_t));

/// Conversion Register (P[1:0] = 00b) [reset = 0000h]
///
/// The 16-bit Conversion register contains the result of the last conversion in binary two's-complement format.
/// Following power-up, the Conversion register is cleared to 0000h, and remains 0000h until the first conversion completes.
struct Conversion {
    /// 16-bit conversion result
    int16_t d = 0x0000;
};

static_assert(sizeof(Conversion) == sizeof(uint16_t));
```

#### Address pointer (see section 7.5.3 of the [datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf))
When writing, the address pointer will always be set to the register being written to.

When reading, it's the user's responsibility to ensure that the address pointer points to the register being read from.

See the following examples:

```cpp
// Point the address pointer to the conversion register.
ads111x::write(
    device_handle,
    AddressPointer {
        .p = AddressPointer::Register::Conversion,
    });


// OK, the address pointer points to the conversion register.
auto conversion = ads111x::read<Conversion>(device_handle).value();

// OK, the address pointer still points to the conversion register.
auto another_conversion = ads111x::read<Conversion>(device_handle).value();

// ads111x::write(
//     *handle_or_error,
//     AddressPointer {
//         .p = AddressPointer::Register::Config,
//     });

// Very bad! The address pointer does not point to the config register, the values in this object will not represent the current config.
// Uncomment the lines above to make this OK.
auto config = ads111x::read<Config>(device_handle).value();
```
```cpp
// Point the address pointer to the conversion register.
ads111x::write(
    device_handle,
    AddressPointer {
        .p = AddressPointer::Register::Conversion,
    });


// OK, the address pointer points to the conversion register.
auto conversion = ads111x::read<Conversion>(device_handle).value();

// Reset the config to the default values, this will set the address pointer to the config register.
ads111x::write(device_handle, Config {});

// ads111x::write(
//     *handle_or_error,
//     AddressPointer {
//         .p = AddressPointer::Register::Conversion,
//     });

// Very bad! The address pointer is now set to the config register because of the write.
// Uncomment the lines above to make this OK.
auto another_conversion = ads111x::read<Conversion>(device_handle).value();
```
