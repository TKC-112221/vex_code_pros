/**
 * main.h  --  PROS project umbrella header.
 *
 * Standard PROS entry-point declarations plus the project's global includes.
 * `api.h` and the `pros/` headers are provided by the PROS kernel (add them by
 * generating/applying the PROS template -- see PROS_SETUP.md).
 */
#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

// Allow the short names (Motor, Imu, ...) and unit literals.
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here.
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#endif  // _PROS_MAIN_H_
