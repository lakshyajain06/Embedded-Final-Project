#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

constexpr size_t sampleRateHz = 50;
constexpr size_t windowSamples = 3 * sampleRateHz;
constexpr float triggerAccelRms = 0.22f;
constexpr float triggerGyroRms = 45.0f;
constexpr float triggerCorrelation = 0.50f;

struct MotionSample {
  int16_t accel[3];  // Raw counts, 0.244 mg/count.
  int16_t gyro[3];   // Raw counts, 0.035 degrees/second/count.
};

struct MotionFeatures {
  float accelRms = 0;
  float gyroRms = 0;
  float correlation = 0;
};

class MotionDetector {
 public:
  void resetWindow() {
    next = 0;
    count = 0;
    sinceAnalysis = 0;
    candidateWindows = 0;
    quietWindows = 0;
  }

  bool add(const MotionSample& sample) {
    samples[next] = sample;
    next = (next + 1) % windowSamples;
    if (count < windowSamples) {
      ++count;
      if (count < windowSamples) return false;
    } else if (++sinceAnalysis < sampleRateHz / 2) {
      return false;
    }
    sinceAnalysis = 0;

    measureEnergy(latest.accelRms, latest.gyroRms);
    latest.correlation = 0;
    if (latest.accelRms >= triggerAccelRms) latest.correlation = measureRhythm();
    if (latched) {
      // Six quiet decisions, spaced half a second apart, before another alert.
      quietWindows = latest.accelRms < 0.12f && latest.gyroRms < 25.0f
                         ? quietWindows + 1 : 0;
      if (quietWindows >= 6) {
        latched = false;
        quietWindows = 0;
      }
      return false;
    }

    // Demonstration thresholds for sustained vigorous, rhythmic wrist motion.
    const bool candidate = latest.accelRms >= triggerAccelRms &&
                           latest.gyroRms >= triggerGyroRms &&
                           latest.correlation >= triggerCorrelation;
    candidateWindows = candidate ? candidateWindows + 1 : 0;
    if (candidateWindows < 2) return false;
    candidateWindows = 0;
    latched = true;
    return true;
  }

  const MotionSample& at(size_t i) const {
    return samples[(next + i) % windowSamples];
  }
  const MotionFeatures& features() const { return latest; }

 private:
  MotionSample samples[windowSamples]{};
  MotionFeatures latest;
  float accelMean[3]{};
  size_t next = 0;
  size_t count = 0;
  size_t sinceAnalysis = 0;
  unsigned candidateWindows = 0;
  unsigned quietWindows = 0;
  bool latched = false;

  void measureEnergy(float& accelRms, float& gyroRms) {
    float accelVariance = 0, gyroVariance = 0;
    for (size_t axis = 0; axis < 3; ++axis) {
      float accelSum = 0, accelSquares = 0, gyroSum = 0, gyroSquares = 0;
      for (const MotionSample& sample : samples) {
        const float a = sample.accel[axis];
        const float g = sample.gyro[axis];
        accelSum += a;
        accelSquares += a * a;
        gyroSum += g;
        gyroSquares += g * g;
      }
      accelMean[axis] = accelSum / windowSamples;
      const float gyroMean = gyroSum / windowSamples;
      accelVariance += accelSquares / windowSamples - accelMean[axis] * accelMean[axis];
      gyroVariance += gyroSquares / windowSamples - gyroMean * gyroMean;
    }
    accelRms = sqrtf(fmaxf(0, accelVariance)) * 0.000244f;
    gyroRms = sqrtf(fmaxf(0, gyroVariance)) * 0.035f;
  }

  float measureRhythm() const {
    // Look for a reversal followed by a repeat after roughly one 2–6 Hz cycle.
    bool reversed = false;
    float best = 0;
    for (size_t lag = 4; lag <= 25; ++lag) {
      float dot = 0, firstPower = 0, secondPower = 0;
      for (size_t i = lag; i < windowSamples; ++i) {
        const MotionSample& first = at(i - lag);
        const MotionSample& second = at(i);
        for (size_t axis = 0; axis < 3; ++axis) {
          const float a = first.accel[axis] - accelMean[axis];
          const float b = second.accel[axis] - accelMean[axis];
          dot += a * b;
          firstPower += a * a;
          secondPower += b * b;
        }
      }
      const float scale = sqrtf(firstPower * secondPower);
      if (dot < -0.2f * scale) reversed = true;
      if (reversed && lag >= 8 && scale > 0) best = fmaxf(best, dot / scale);
    }
    return best;
  }
};
