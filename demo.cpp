#include "autofocus_cpp/solution.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

// A smooth focus peak and a small, repeatable ripple make a standalone demo.
constexpr int kLength = 480;
constexpr int kPeakCenter = 173;

std::vector<std::int32_t> MakeSharpnessCurve()
{
    std::vector<std::int32_t> values;
    values.reserve(kLength);
    for (int position = 0; position < kLength; ++position) {
        const double offset = (position - kPeakCenter) / 21.0;
        const int ripple = (position * 17) % 13;
        values.push_back(120 + ripple + static_cast<int>(
            std::lround(4500.0 * std::exp(-0.5 * offset * offset))));
    }
    return values;
}

const auto sharpness = MakeSharpnessCurve();

struct Motor {
    int position = 0;
    int rightmost = 0;
    int direction = 0;
    int queries = 0;
    int travel = 0;
    int reversals = 0;
};

Motor motor;

}  // namespace

std::int32_t RunMotor(std::uint32_t index)
{
    if (index >= sharpness.size()) {
        std::cerr << "Invalid motor position " << index << '\n';
        std::exit(EXIT_FAILURE);
    }
    const int next = static_cast<int>(index);
    const int delta = next - motor.position;
    if (delta != 0) {
        const int direction = delta > 0 ? 1 : -1;
        motor.reversals += motor.direction != 0 && motor.direction != direction;
        motor.direction = direction;
    }
    motor.position = next;
    motor.rightmost = std::max(motor.rightmost, next);
    ++motor.queries;
    motor.travel += std::abs(delta);
    return sharpness[index];
}

int main()
{
    const auto peak = std::max_element(sharpness.begin(), sharpness.end());
    const int target = static_cast<int>(peak - sharpness.begin());
    if (FindPeak(static_cast<std::uint32_t>(sharpness.size())) != 0) {
        std::cerr << "Focus search failed\n";
        return EXIT_FAILURE;
    }

    const int error = std::abs(motor.position - target);
    const int overshoot = std::max(0, motor.rightmost - target);
    const int cost = motor.queries + motor.travel;
    const int score = error > 3 || motor.queries >= 200 || motor.reversals >= 7
        || overshoot >= 200
        ? 0
        : std::max(0, 3000 - 200 * error - cost - 30 * motor.reversals - overshoot);

    std::cout << "Target index: " << target << '\n'
              << "Final index: " << motor.position << '\n'
              << "Motor queries: " << motor.queries << '\n'
              << "Motor travel: " << motor.travel << '\n'
              << "Direction reversals: " << motor.reversals << '\n'
              << "Overshoot: " << overshoot << '\n'
              << "Simulated score: " << score << " / 3000\n";
}
