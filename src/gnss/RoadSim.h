#pragma once
#include <stdint.h>

// Road-following drive simulator ("mô phỏng trên đường thật", 2026-09-29):
// with no GNSS module fitted, it drives a virtual car along the REAL road
// network in the microSD speed map — segment to segment through shared
// nodes, obeying one-way directions, choosing a continuation at every
// junction (mostly the straightest, sometimes a turn, staying on bigger
// roads), cruising at each segment's own speed limit and slowing for turns.
// Everything downstream (matcher, camera/sign warnings, voice, map) sees it
// exactly like a real fix, flagged as simulated via GnssSnapshot::fixSeq's
// top bit so the Dashboard can badge it "SIM".
//
// Driven from gnss/GNSS.cpp's task (all SD reads happen there). Start/stop
// requests may come from any task.

// Start near (lat,lon): snaps to the nearest drivable segment. speedFactor
// scales the cruise speed vs. the limit (1.0 = at the limit, 1.15 = 15% over
// to exercise overspeed alerts).
void roadSimRequestStart(float lat, float lon, float speedFactor);
void roadSimRequestStop();
bool roadSimActive();

// Called by the GNSS task every loop; advances the car and returns true (with
// the simulated fix) while active.
bool roadSimStep(float *lat, float *lon, float *headingDeg, float *speedKmh);

// Default start point for the automatic no-GPS demo: Hoàng Quốc Việt, Hà Nội.
static const float kRoadSimDefaultLat = 21.04660f;
static const float kRoadSimDefaultLon = 105.78520f;
