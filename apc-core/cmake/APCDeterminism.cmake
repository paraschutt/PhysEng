# APCDeterminism.cmake
# Detect platform floating-point mode
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(APC_FP_MODE "X64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(APC_FP_MODE "ARM64")
else()
    set(APC_FP_MODE "CONSOLE")
endif()
