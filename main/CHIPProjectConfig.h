#pragma once

// ============================================================
// Matter (CHIP) Project Configuration for LK ICS2 Bridge
//
// IMPORTANT — keep this file minimal.
//
// esp-matter maps many CHIP_DEVICE_CONFIG_* macros directly to
// Kconfig symbols (e.g. CHIP_DEVICE_CONFIG_ENABLE_THREAD becomes
// CONFIG_ENABLE_MATTER_OVER_THREAD).  If we redefine the same macro
// here, GCC warns about the token-string mismatch and -Werror turns
// that warning into a hard build error.
//
// Rule: only define things here that have NO Kconfig equivalent in
// esp-matter.  Everything else belongs in sdkconfig.defaults.
// Use #ifndef guards so the SDK's value always wins if it exists.
// ============================================================

// Logging
#ifndef CHIP_DEVICE_CONFIG_LOG_PROVISIONING
#define CHIP_DEVICE_CONFIG_LOG_PROVISIONING     1
#endif

// Failsafe window (seconds).  No direct Kconfig equivalent.
#ifndef CHIP_DEVICE_CONFIG_FAILSAFE_EXPIRY_LENGTH
#define CHIP_DEVICE_CONFIG_FAILSAFE_EXPIRY_LENGTH   60
#endif

// Maximum number of commissioner fabrics.  No direct Kconfig equivalent.
#ifndef CHIP_CONFIG_MAX_FABRICS
#define CHIP_CONFIG_MAX_FABRICS                 5
#endif

// Thread task stack size (bytes).  No direct Kconfig equivalent.
#ifndef CHIP_DEVICE_CONFIG_THREAD_TASK_STACK_SIZE
#define CHIP_DEVICE_CONFIG_THREAD_TASK_STACK_SIZE   8192
#endif
