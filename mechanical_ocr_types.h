#pragma once

#include <Arduino.h>

#define MACHINESENS_MECH_TYPES_V4 1

// ============================================================================
// Mechanical OCR shared types
//
// This small header exists because Arduino automatically inserts function
// prototypes before normal .ino declarations. Keeping these custom types here
// makes them visible before those generated prototypes.
// ============================================================================

constexpr int MECH_MAX_DIGITS = 16;
constexpr int MECH_MAX_COMPONENTS = 48;
constexpr int MECH_FLOOD_STACK_CAP = 32768;

struct MechanicalComponent {
  int x;
  int y;
  int w;
  int h;
  int area;
};

struct MechanicalDigitResult {
  char digit;
  float score;
  char secondDigit;
  float secondScore;
  MechanicalComponent component;
};

struct MechanicalOcrResult {
  bool success;
  char reading[40];
  char raw[32];
  int expectedDigits;
  int detectedDigits;
  float averageScore;
  float averageMargin;
  int saturationLimitUsed;
  bool rgb565RescueUsed;
  bool whiteInkMaskUsed;
  MechanicalDigitResult digits[MECH_MAX_DIGITS];
  char error[128];
};
