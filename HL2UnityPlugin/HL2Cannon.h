#pragma once
#include "HL2Cannon.g.h"

// Note: Remove this static_assert after copying these generated source files to your project.
// This assertion exists to avoid compiling these generated source files directly.
static_assert(false, "Do not compile generated C++/WinRT source files directly");

namespace winrt::HL2UnityPlugin::implementation
{
    struct HL2Cannon : HL2CannonT<HL2Cannon>
    {
        HL2Cannon() = default;

        void StartUpdateLoop();
        void StopUpdateLoop();
    };
}
namespace winrt::HL2UnityPlugin::factory_implementation
{
    struct HL2Cannon : HL2CannonT<HL2Cannon, implementation::HL2Cannon>
    {
    };
}
