// apc_fp_mode_arm64.cpp
#include "apc_fp_mode.h"
namespace apc {
    void EnforceDeterministicFP() {
        // ARM64: set FPCR to round-to-nearest, flush-to-zero off
        // (stub ? real implementation uses asm)
    }
}
