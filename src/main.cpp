#include <Arduino.h>
#include "LfHal.h"
#include "defines.h"
#include "parameters.h"

#include <cstdint>
#include <algorithm>

bool led = false;

template <class Hal>
int_fast8_t findLine() {
    using LineSensorT = typename Hal::LineSensorT;
    typename Hal::LineSensorBufferT buffer;

    constexpr auto& kernel = Parameters::LineDetector::detectionKernel;
    constexpr int32_t kernelSize = sizeof(kernel) / sizeof(kernel[0]);
    constexpr int32_t bufferSize = std::tuple_size<decltype(buffer)>::value;

    LineSensorT minValue, maxValue;
    std::tie(minValue, maxValue) = Hal::instance().readLineSensor(buffer);

    constexpr auto valueSign = Parameters::LineDetector::lineType == LineType::WhiteOnBlack ? 1 : -1;
    auto valueOffset = Parameters::LineDetector::lineType == LineType::WhiteOnBlack ? minValue : maxValue;

    auto bestWeight = std::numeric_limits<int32_t>::min();
    auto bestPosition = std::numeric_limits<int32_t>::min();

    for (int32_t offsetIndex = 1 - kernelSize;
         offsetIndex < bufferSize;
         ++offsetIndex)
    {
        const size_t min = std::max<int32_t>(-offsetIndex, 0);
        const size_t max = std::min<int32_t>(bufferSize - offsetIndex, kernelSize);

        LineSensorT weight = 0;

        for (size_t i = min; i < max; ++i)
        {
            auto value = valueSign * (buffer[offsetIndex + i] - valueOffset);
            weight += kernel[i] * value;
        }

        if (weight > bestWeight)
        {
            bestWeight = weight;
            bestPosition = offsetIndex;
        }
    }

    static_assert((kernelSize - bufferSize) % 2 == 0, "Kernel size must be even iff buffer size is even");
    return bestPosition - (bufferSize - kernelSize) / 2;
}

void setup() {
    Serial.begin(9600);

    while (!Serial) {
      LfHal::instance().setBuiltinLed(true);
      delay(50);
      LfHal::instance().setBuiltinLed(false);
      delay(50);
    }
}

void loop() {
    LfHal::instance().setBuiltinLed(led);
    led = !led;

    delay(1000);

    Serial.println("<tick>");
    Serial.println(findLine<LfHal>());

    Serial.println("</tick>");
}
