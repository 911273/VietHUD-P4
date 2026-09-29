#pragma once
#include <stdint.h>

struct TouchPoint {
    uint16_t x = 0, y = 0;
    bool pressed = false;
};

// Owns the AXS15231BTouch driver and samples it continuously on its own
// FreeRTOS task (Core 0), independent of the UI/render task on Core 1 and
// LVGL's own indev poll period. Rationale: the UI task blocks for ~33ms per
// rendered frame (confirmed hardware ceiling — see
// docs/V1.2_hardening_proposal.md), during which the old design (touch read
// happening inside the same task as the render) couldn't sample touch at
// all. Sampling on a separate core means the shared point stays fresh even
// while a flush is in progress; the LVGL indev callback just reads whatever
// is currently there via touchSnapshot(), never touching the I2C bus itself.
void touchTaskStart();
TouchPoint touchSnapshot();

#ifdef VIETHUD_P4
// Touch calibration (touch/TouchTaskP4.cpp): 5-point affine fit, saved in NVS.
extern volatile bool gTouchCalRequested; // set from UI/serial; main loop runs it
bool touchCalValid();                     // calibration exists for the current rotation
bool touchCalibrationRun(bool firstBoot); // blocking; firstBoot = give up after 30 s idle
#endif
