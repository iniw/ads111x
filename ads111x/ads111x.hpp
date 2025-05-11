// https://www.ti.com/lit/ds/symlink/ads1115.pdf

#pragma once

#include <sdkconfig.h>

#if !defined(CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2) || CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2 == 0
#    error "This component requires v2 of ESP-IDF's I2C driver to be enabled. Add `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2=y` to your sdkconfig."
#endif

#include <expected>

#include <driver/i2c_master.h>

namespace ads111x {

namespace reg {

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

/// Config Register (P[1:0] = 01b) [reset = 8583h]
///
/// The 16-bit Config register controls the operating mode, input selection, data rate, full-scale range, and comparator modes.
struct [[gnu::packed]] Config {
    /// Comparator queue and disable (ADS1114 and ADS1115 only)
    enum class ComparatorQueue : unsigned {
        /// Assert after one conversion
        AssertAfterOneConversion = 0b00,
        /// Assert after two conversions
        AssertAfterTwoConversions = 0b01,
        /// Assert after four conversions
        AssertAfterFourConversions = 0b10,
        /// Disable comparator and set ALERT/RDY pin to high-impedance (default)
        DisableComparator = 0b11,
    }
    /// These bits perform two functions. When set to 11, the comparator is disabled and the ALERT/RDY pin is set to a high-impedance state. When set to any other value, the ALERT/RDY pin and the comparator function are enabled, and the set value determines the number of successive conversions exceeding the upper or lower threshold required before asserting the ALERT/RDY pin.
    /// These bits serve no function on the ADS1113.
    comp_que : 2
        = ComparatorQueue::DisableComparator;

    /// Latching comparator (ADS1114 and ADS1115 only)
    enum class LatchingComparator : unsigned {
        /// Nonlatching comparator. The ALERT/RDY pin does not latch when asserted (default).
        NonLatching,
        ///  Latching comparator. The asserted ALERT/RDY pin remains latched until conversion data are read by the controller or an appropriate SMBus alert response is sent by the controller. The device responds with an address, and is the lowest address currently asserting the ALERT/RDY bus line.
        Latching,
    }
    /// This bit controls whether the ALERT/RDY pin latches after being asserted or clears after conversions are within the margin of the upper and lower threshold values.
    /// This bit serves no function on the ADS1113.
    comp_lat : 1
        = LatchingComparator::NonLatching;

    /// Comparator polarity (ADS1114 and ADS1115 only)
    enum class ComparatorPolarity : unsigned {
        ///  Active low (default)
        ActiveLow = 0b0,
        ///  Active high
        ActiveHigh = 0b1,
    }
    /// This bit controls the polarity of the ALERT/RDY pin.
    /// This bit serves no function on the ADS1113.
    comp_pol : 1
        = ComparatorPolarity::ActiveLow;

    /// Comparator mode (ADS1114 and ADS1115 only)
    enum class ComparatorMode : unsigned {
        /// Traditional comparator (default)
        TraditionalComparator = 0b0,
        /// Window comparator
        WindowComparator = 0b1,
    }
    /// This bit configures the comparator operating mode.
    /// This bit serves no function on the ADS1113.
    comp_mode : 1
        = ComparatorMode::TraditionalComparator;

    /// Data rate
    enum class DataRate : unsigned {
        /// 8SPS
        _8SPS = 0b000,
        /// 16SPS
        _16SPS = 0b001,
        /// 32SPS
        _32SPS = 0b010,
        /// 64SPS
        _64SPS = 0b011,
        /// 128SPS
        _128SPS = 0b100,
        /// 250SPS
        _250SPS = 0b101,
        /// 475SPS
        _475SPS = 0b110,
        /// 860SPS
        _860SPS = 0b111,
    }
    /// These bits control the data rate setting.
    dr : 3
        = DataRate::_128SPS;

    /// Device operating mode
    enum class Mode : unsigned {
        ///  Continuous-conversion mode
        ContinuousConversion = 0b0,
        /// Single-shot mode or power-down state (default)
        SingleShot = 0b1,
    }
    /// This bit controls the operating mode.
    mode : 1
        = Mode::SingleShot;

    /// Programmable gain amplifier configuration
    enum class PGA : unsigned {
        /// FSR = ±6.144V
        FSR_6_144V = 0b000,
        /// FSR = ±4.096V
        FSR_4_096V = 0b001,
        /// FSR = ±2.048V
        FSR_2_048V = 0b010,
        /// FSR = ±1.024V
        FSR_1_024V = 0b011,
        /// FSR = ±0.512V
        FSR_0_512V = 0b100,
        /// FSR = ±0.256V
        FSR_0_256V = 0b101,
        /// FSR = ±0.256V
        FSR_0_256V_1 = 0b110,
        /// FSR = ±0.256V
        FSR_0_256V_2 = 0b111,
    }
    /// These bits set the FSR of the programmable gain amplifier.
    /// These bits serve no function on the ADS1113. ADS1113 always uses FSR = ±2.048V.
    pga : 3
        = PGA::FSR_2_048V;

    /// Input multiplexer configuration (ADS1115 only)
    enum class Mux : unsigned {
        /// AINP = AIN0 and AINN = AIN1 (default)
        AINP_AIN0_AINN_AIN1 = 0b000,
        /// AINP = AIN0 and AINN = AIN3
        AINP_AIN0_AINN_AIN3 = 0b001,
        /// AINP = AIN1 and AINN = AIN3
        AINP_AIN1_AINN_AIN3 = 0b010,
        /// AINP = AIN2 and AINN = AIN3
        AINP_AIN2_AINN_AIN3 = 0b011,
        /// AINP = AIN0 and AINN = GND
        AINP_AIN0_AINN_GND = 0b100,
        /// AINP = AIN1 and AINN = GND
        AINP_AIN1_AINN_GND = 0b101,
        /// AINP = AIN2 and AINN = GND
        AINP_AIN2_AINN_GND = 0b110,
        /// AINP = AIN3 and AINN = GND
        AINP_AIN3_AINN_GND = 0b111,
    }
    /// These bits configure the input multiplexer.
    /// These bits serve no function on the ADS1113 and ADS1114. ADS1113 and ADS1114 always use inputs AINP = AIN0 and AINN = AIN1.
    mux : 3
        = Mux::AINP_AIN0_AINN_AIN1;

    /// Operational status or single-shot conversion start
    enum class OS : unsigned {
        // When writing:

        /// No effect.
        NoEffect = 0b0,
        /// Start a single conversion (when in power-down state).
        StartSingleConversion = 0b1,

        // When reading:

        /// Device is currently performing a conversion.
        PerformingConversion = 0b0,
        /// Device is not currently performing a conversion.
        NotPerformingConversion = 0b1,
    }
    /// This bit determines the operational status of the device.
    /// OS can only be written when in power-down state and has no effect when a conversion is ongoing.
    os : 1
        = OS::NotPerformingConversion;
};

static_assert(sizeof(Config) == sizeof(uint16_t));

/// Lo_thresh (P[1:0] = 10b) [reset = 8000h] and Hi_thresh (P[1:0] = 11b) [reset = 7FFFh] Registers
///
/// These two registers are applicable to the ADS1115 and ADS1114. These registers serve no purpose in the ADS1113. The upper and lower threshold values used by the comparator are stored in two 16-bit registers in 2's complement format. The comparator is implemented as a digital comparator; therefore, the values in these registers must be updated whenever the PGA settings are changed.
/// The conversion-ready function of the ALERT/RDY pin is enabled by setting the Hi_thresh register MSB to 1b and the Lo_thresh register MSB to 0b. To use the comparator function of the ALERT/RDY pin, the Hi_thresh register value must always be greater than the Lo_thresh register value. The threshold register formats are shown in Figure 8-6. When set to RDY mode, the ALERT/RDY pin outputs the OS bit when in single-shot mode, and provides a continuous-conversion ready pulse when in continuous-conversion mode
struct Lo_thresh {
    /// Low threshold value
    int16_t lo_thresh = 0x8000;
};

static_assert(sizeof(Lo_thresh) == sizeof(uint16_t));

/// Lo_thresh (P[1:0] = 10b) [reset = 8000h] and Hi_thresh (P[1:0] = 11b) [reset = 7FFFh] Registers
///
/// These two registers are applicable to the ADS1115 and ADS1114. These registers serve no purpose in the ADS1113. The upper and lower threshold values used by the comparator are stored in two 16-bit registers in 2's complement format. The comparator is implemented as a digital comparator; therefore, the values in these registers must be updated whenever the PGA settings are changed.
/// The conversion-ready function of the ALERT/RDY pin is enabled by setting the Hi_thresh register MSB to 1b and the Lo_thresh register MSB to 0b. To use the comparator function of the ALERT/RDY pin, the Hi_thresh register value must always be greater than the Lo_thresh register value. The threshold register formats are shown in Figure 8-6. When set to RDY mode, the ALERT/RDY pin outputs the OS bit when in single-shot mode, and provides a continuous-conversion ready pulse when in continuous-conversion mode
struct Hi_thresh {
    /// High threshold value
    int16_t hi_thresh = 0x7FFF;
};

static_assert(sizeof(Hi_thresh) == sizeof(uint16_t));

}

/// I2C Address Selection
///
/// The ADS111x have one address pin, ADDR, that configures the I2C address of the device. This pin can be connected to GND, VDD, SDA, or SCL, allowing for four different addresses to be selected with one pin, as shown in Table 7-2. The state of address pin ADDR is sampled continuously. Use the GND, VDD and SCL addresses first. If SDA is used as the device address, hold the SDA line low for at least 100 ns after the SCL line goes low to make sure the device decodes the address correctly during I2C communication.
enum class AddrSelection : uint16_t {
    GND = 0b1001000,
    VDD = 0b1001001,
    SDA = 0b1001010,
    SCL = 0b1001011,
};

std::expected<i2c_master_dev_handle_t, esp_err_t> init(i2c_master_bus_handle_t, AddrSelection, uint32_t scl_frequency);

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t, reg::AddressPointer);

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t, reg::Config);

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t, reg::Lo_thresh);

std::expected<void, esp_err_t> write(i2c_master_dev_handle_t, reg::Hi_thresh);

template<typename T>
std::expected<T, esp_err_t> read(i2c_master_dev_handle_t) {
    static_assert(false, "Invalid register for `read()`");
}

template<>
std::expected<reg::Conversion, esp_err_t> read<reg::Conversion>(i2c_master_dev_handle_t);

template<>
std::expected<reg::Config, esp_err_t> read<reg::Config>(i2c_master_dev_handle_t);

template<>
std::expected<reg::Lo_thresh, esp_err_t> read<reg::Lo_thresh>(i2c_master_dev_handle_t);

template<>
std::expected<reg::Hi_thresh, esp_err_t> read<reg::Hi_thresh>(i2c_master_dev_handle_t);

}
