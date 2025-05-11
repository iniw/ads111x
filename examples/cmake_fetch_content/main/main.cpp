// This code is just to show that things will compile :)

#include <ads111x/ads111x.hpp>

extern "C" void app_main() {
    static_assert(sizeof(ads111x::reg::AddressPointer) == sizeof(uint8_t));
}
