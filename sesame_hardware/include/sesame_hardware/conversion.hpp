// Copyright 2026 stregatto
// SPDX-License-Identifier: Apache-2.0
//
// Conversione tra angoli dell'URDF e angoli del servo (gradi), senza dipendenze ROS.
//
//   servo = 90 + direzione * (angolo - 90) + subtrim
//   angolo = 90 + (servo - 90 - subtrim) / direzione      (direzione = +1 o -1)

#ifndef SESAME_HARDWARE__CONVERSION_HPP_
#define SESAME_HARDWARE__CONVERSION_HPP_

#include <algorithm>
#include <cmath>

namespace sesame_hardware
{

constexpr double kPi = 3.14159265358979323846;

/// Calibrazione di un giunto, letta da calibration/default.yaml tramite lo xacro.
struct JointCalibration
{
  int firmware_index{-1};  ///< posizione nel messaggio (R1, R2, L1, L2, R4, R3, L3, L4)
  double direction{1.0};   ///< +1 o -1
  double subtrim_deg{0.0};
  double min_deg{0.0};     ///< limiti in gradi URDF
  double max_deg{180.0};
};

inline double rad_to_deg(double rad) { return rad * 180.0 / kPi; }
inline double deg_to_rad(double deg) { return deg * kPi / 180.0; }

/// Gradi URDF -> gradi del servo.
inline double urdf_to_servo_deg(double urdf_deg, const JointCalibration & cal)
{
  return 90.0 + cal.direction * (urdf_deg - 90.0) + cal.subtrim_deg;
}

/// Gradi del servo -> gradi URDF.
inline double servo_to_urdf_deg(double servo_deg, const JointCalibration & cal)
{
  return 90.0 + (servo_deg - 90.0 - cal.subtrim_deg) / cal.direction;
}

/// Limita value a [low, high]; clamped diventa true se il valore e' stato cambiato.
inline double clamp_flag(double value, double low, double high, bool & clamped)
{
  const double out = std::min(std::max(value, low), high);
  if (out != value) {
    clamped = true;
  }
  return out;
}

}  // namespace sesame_hardware

#endif  // SESAME_HARDWARE__CONVERSION_HPP_
