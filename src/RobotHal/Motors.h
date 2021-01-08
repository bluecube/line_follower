#pragma once

#include <driver/mcpwm.h>

#include <cstdint>
#include <utility>
#include <array>

namespace RobotHal {

class Hal;

class Motors {

protected:
    Motors() {}
public:
    Motors(const Motors&) = delete;
    Motors(Motors&&) = delete;
    Motors& operator=(const Motors&) = delete;
    Motors& operator=(Motors&&) = delete;

    // Using float percent is an artefat of ESP-IDF mcpwm module
    using PwmT = float;
    static constexpr PwmT motorMaxValue = 100.0f;

    /// Set PWM signals for motors. Positive values mean driving forward,
    void set(PwmT left, PwmT right) {
        set(MCPWM_UNIT_0, left);
        set(MCPWM_UNIT_1, right);
    }

    /// Reconfigure the motors so that they can't drive and only make noise.
    /// See setBeepTone() for description of the pitch parameters.
    void startBeep(uint8_t leftPitch, uint8_t rightPitch);

    /// Set pitch of the motor beeping.
    /// Uses MIDI note numbers for pitch values:
    /// chromatic scale starting at C five octaves below middle C (8.1758 Hz),
    /// going up a octave / for every 12 units.
    /// Value 69 corresponds to concert A (440 Hz)
    void setBeepTone(uint8_t leftPitch, uint8_t rightPitch);

    /// Stop the beeping sound, reset motor to default settings and zero speed.
    void stopBeep() {
        setupMotors();
    }

    std::pair<int16_t, int16_t> readEncoders() const;

protected:
    /// Convert pitch value to frequency in Hz.
    /// See setBeepTone() for description of the tuning.
    static constexpr uint32_t beepPitchToFrequency(uint8_t pitch);

    static void setup();
    static void setupMotors();

    static void set(mcpwm_unit_t unit, float duty);


    friend class Hal;
};

}
