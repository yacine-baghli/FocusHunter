#include "solution.h"

// Adaptive autofocus search with a stopping rule for low-prominence peaks.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <set>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

// The challenge harness or local demo supplies this motor interface.
extern int32_t RunMotor(uint32_t idx);

namespace {

using std::size_t;

constexpr int kMaxQueries = 199;
constexpr int64_t kModelRadius = 8;

using Position = int64_t;
using Value = int64_t;

Value Median(std::vector<Value> values)
{
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

std::pair<double, double> Levels(const std::vector<Value>& values)
{
    std::vector<Value> ordered = values;
    std::sort(ordered.begin(), ordered.end());
    const size_t baselineIndex = std::max<size_t>(0, values.size() / 5) == 0
        ? 0
        : values.size() / 5 - 1;

    std::vector<Value> differences;
    differences.reserve(values.size() > 0 ? values.size() - 1 : 0);
    for (size_t index = 1; index < values.size(); ++index) {
        differences.push_back(std::llabs(values[index] - values[index - 1]));
    }
    const double noise = differences.empty()
        ? 1.0
        : std::max(1.0, static_cast<double>(Median(std::move(differences))));
    return {static_cast<double>(ordered[baselineIndex]), noise};
}

double EstimatedPeakDistance(
    const std::vector<Position>& positions,
    const std::vector<Value>& values,
    double baseline)
{
    constexpr size_t sampleCount = 7;
    const size_t first = positions.size() - sampleCount;
    const Position current = positions.back();

    std::vector<double> recentLogs;
    recentLogs.reserve(sampleCount);
    for (size_t index = first; index < values.size(); ++index) {
        recentLogs.push_back(
            std::log(std::max(1.0, static_cast<double>(values[index]) - baseline)));
    }

    std::vector<double> midpoints;
    std::vector<double> slopes;
    midpoints.reserve(sampleCount - 1);
    slopes.reserve(sampleCount - 1);
    for (size_t index = 1; index < sampleCount; ++index) {
        const Position left = positions[first + index - 1];
        const Position right = positions[first + index];
        midpoints.push_back(
            static_cast<double>(left + right) / 2.0
            - static_cast<double>(current));
        slopes.push_back(
            (recentLogs[index] - recentLogs[index - 1])
            / static_cast<double>(right - left));
    }

    double meanX = 0.0;
    double meanSlope = 0.0;
    for (double point : midpoints) {
        meanX += point;
    }
    for (double slope : slopes) {
        meanSlope += slope;
    }
    meanX /= static_cast<double>(midpoints.size());
    meanSlope /= static_cast<double>(slopes.size());

    double denominator = 0.0;
    for (double point : midpoints) {
        denominator += (point - meanX) * (point - meanX);
    }
    if (denominator <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    double numerator = 0.0;
    for (size_t index = 0; index < midpoints.size(); ++index) {
        numerator += (midpoints[index] - meanX) * (slopes[index] - meanSlope);
    }
    const double curvature = numerator / denominator;
    const double derivative = meanSlope - curvature * meanX;
    if (curvature >= 0.0 || derivative <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return -derivative / curvature;
}

struct SupportedPeak {
    size_t index;
    double level;
    double baseline;
};

SupportedPeak BestSupportedPeak(
    const std::vector<Position>& /* positions */,
    const std::vector<Value>& values)
{
    std::vector<Value> ordered = values;
    std::sort(ordered.begin(), ordered.end());
    const size_t baselineIndex = values.size() / 5 == 0 ? 0 : values.size() / 5 - 1;
    const double baseline = static_cast<double>(ordered[baselineIndex]);
    if (values.size() == 1) {
        return {0, static_cast<double>(values[0]), baseline};
    }

    size_t bestIndex = 0;
    double bestSupport =
        (2.0 * static_cast<double>(values[0]) + static_cast<double>(values[1])) / 3.0;
    for (size_t index = 1; index + 1 < values.size(); ++index) {
        const double support = (
            static_cast<double>(values[index - 1])
            + 2.0 * static_cast<double>(values[index])
            + static_cast<double>(values[index + 1])) / 4.0;
        if (support > bestSupport) {
            bestIndex = index;
            bestSupport = support;
        }
    }
    return {bestIndex, bestSupport, baseline};
}

bool DensePeakFinished(
    const std::vector<Value>& values,
    const std::vector<Position>& positions,
    Position bestIndex,
    Value bestValue)
{
    if (positions.back() - bestIndex < 4 || values.size() < 7) {
        return false;
    }
    const auto [baseline, noise] = Levels(values);
    (void)noise;
    const double prominence = static_cast<double>(bestValue) - baseline;
    if (prominence <= 0.0) {
        return false;
    }
    const size_t count = values.size();
    const double tail = (
        static_cast<double>(values[count - 3])
        + static_cast<double>(values[count - 2])
        + static_cast<double>(values[count - 1])) / 3.0;
    const double decline = (static_cast<double>(bestValue) - tail) / prominence;
    int recentFalls = 0;
    for (size_t index = count - 5; index < count; ++index) {
        recentFalls += values[index] < values[index - 1] ? 1 : 0;
    }
    return (decline >= 0.18 && recentFalls >= 3)
        || positions.back() - bestIndex >= 120;
}

int ScanStep(Position position, const std::vector<Value>& values)
{
    int step;
    if (position < 128) {
        step = 4;
    } else if (position < 320) {
        step = 6;
    } else if (position < 560) {
        step = 8;
    } else if (position < 800) {
        step = 10;
    } else {
        step = 12;
    }

    if (values.size() >= 8) {
        const auto [baseline, noise] = Levels(values);
        const double signal = static_cast<double>(
            *std::max_element(values.begin(), values.end())) - baseline;
        if (signal < 10.0 * noise) {
            step += 2;
        } else if (
            signal >= 14.0 * noise
            && static_cast<double>(values.back()) >= baseline + 0.40 * signal) {
            step = std::min(step, 4);
        }
    }
    return step;
}

bool EnterDense(
    const std::vector<Position>& positions,
    const std::vector<Value>& values)
{
    if (values.size() < 9) {
        return false;
    }
    const auto [baseline, noise] = Levels(values);
    const double prominence = static_cast<double>(values.back()) - baseline;

    std::vector<Value> slopes;
    slopes.reserve(values.size() - 1);
    for (size_t index = 1; index < values.size(); ++index) {
        slopes.push_back(values[index] - values[index - 1]);
    }
    int recentRises = 0;
    for (size_t index = slopes.size() - 6; index < slopes.size(); ++index) {
        recentRises += slopes[index] > 0 ? 1 : 0;
    }
    const Value earlierSlope = *std::max_element(slopes.end() - 4, slopes.end() - 1);
    const Value earlierWindowMaximum =
        *std::max_element(slopes.end() - 5, slopes.end() - 2);
    const bool decelerating =
        static_cast<double>(slopes.back()) <= 1.20 * static_cast<double>(earlierSlope)
        && (slopes.back() < earlierSlope
            || slopes[slopes.size() - 2] < earlierWindowMaximum);
    const double estimatedDistance = EstimatedPeakDistance(positions, values, baseline);

    return values.back() == *std::max_element(values.begin(), values.end())
        && recentRises >= 5
        && decelerating
        && estimatedDistance <= 24.0
        && prominence >= 14.0 * noise
        && prominence >= 1.80 * std::max(1.0, baseline);
}

bool NarrowProbeTrigger(
    const std::vector<Position>& positions,
    const std::vector<Value>& values)
{
    if (values.size() < 7 || positions.back() - positions[positions.size() - 2] < 4) {
        return false;
    }
    const std::vector<Value> prior(values.begin(), values.end() - 1);
    const auto [baseline, noise] = Levels(prior);
    const double prominence = static_cast<double>(values.back()) - baseline;
    const double jump = static_cast<double>(values.back() - values[values.size() - 2]);
    return values.back() == *std::max_element(values.begin(), values.end())
        && prominence >= 18.0 * noise
        && prominence >= 1.10 * std::max(1.0, baseline)
        && jump >= 0.32 * prominence;
}

bool ShouldTurn(
    Position arrayLength,
    const std::vector<Position>& positions,
    const std::vector<Value>& values)
{
    if (values.size() < 6) {
        return false;
    }
    const Value maximum = *std::max_element(values.begin(), values.end());
    const Value minimum = *std::min_element(values.begin(), values.end());
    const size_t count = values.size();
    if (
        2 * positions.back() < arrayLength - 1
        && values[count - 1] == values[count - 2]
        && values[count - 2] == values[count - 3]
        && values.back() == maximum
        && values.back() > minimum) {
        return false;
    }

    const SupportedPeak peak = BestSupportedPeak(positions, values);
    if (peak.index >= values.size() - 2) {
        return false;
    }
    const double prominence = peak.level - peak.baseline;
    std::vector<Value> differences;
    differences.reserve(values.size() - 1);
    for (size_t index = 1; index < values.size(); ++index) {
        differences.push_back(std::llabs(values[index] - values[index - 1]));
    }
    std::sort(differences.begin(), differences.end());
    const size_t percentileIndex = differences.size() / 3;
    const double noise = std::max(1.0, static_cast<double>(differences[percentileIndex]));
    if (prominence < 15.0 * noise) {
        return false;
    }

    const double halfHeight = peak.baseline + 0.5 * prominence;
    size_t leftIndex = peak.index;
    while (leftIndex > 0 && static_cast<double>(values[leftIndex - 1]) >= halfHeight) {
        --leftIndex;
    }
    size_t rightIndex = peak.index;
    while (
        rightIndex + 1 < values.size()
        && static_cast<double>(values[rightIndex + 1]) >= halfHeight) {
        ++rightIndex;
    }
    const double widthFraction = prominence >= 25.0 * noise ? 0.03 : 0.05;
    const Position minimumWidth = std::max<Position>(
        16, static_cast<Position>(widthFraction * static_cast<double>(arrayLength)));
    if (positions[rightIndex] - positions[leftIndex] < minimumWidth) {
        return false;
    }

    const double scale = std::max(1.0, peak.baseline);
    if (
        prominence * static_cast<double>(positions[rightIndex] - positions[leftIndex])
        < 0.06 * scale * static_cast<double>(arrayLength)) {
        return false;
    }

    const double tailLevel = (
        static_cast<double>(values[count - 1])
        + static_cast<double>(values[count - 2])) / 2.0;
    const double declineFraction = (peak.level - tailLevel) / prominence;
    int recentFalls = 0;
    const size_t fallStart = std::max<size_t>(1, values.size() - 3);
    for (size_t index = fallStart; index < values.size(); ++index) {
        recentFalls += values[index] < values[index - 1] ? 1 : 0;
    }
    const Position distanceAfterPeak = positions.back() - positions[peak.index];
    if (prominence >= scale) {
        const bool enough = declineFraction >= 0.25 && recentFalls >= 2;
        return (enough && distanceAfterPeak >= 12) || distanceAfterPeak >= 120;
    }
    const bool enough = declineFraction >= 0.45 && recentFalls >= 3;
    if (prominence < 0.90 * scale) {
        const auto rawBest = std::max_element(values.begin(), values.end());
        const size_t rawBestIndex = static_cast<size_t>(
            std::distance(values.begin(), rawBest));
        if (peak.index != rawBestIndex) {
            return false;
        }

        int localPeakCount = 0;
        for (size_t index = 1; index + 1 < values.size(); ++index) {
            if (values[index] >= values[index - 1]
                && values[index] >= values[index + 1]) {
                ++localPeakCount;
            }
        }
        const double localPeakRate = static_cast<double>(localPeakCount)
            / static_cast<double>(values.size());
        const double span = std::max(
            1.0, static_cast<double>(maximum - minimum));
        const double startNormalized = (
            static_cast<double>(values.front()) - peak.baseline) / span;
        const double peakFraction = static_cast<double>(positions[peak.index])
            / static_cast<double>(arrayLength);
        const double observedWidthFraction = static_cast<double>(
            positions[rightIndex] - positions[leftIndex])
            / static_cast<double>(arrayLength);

        return enough
            && distanceAfterPeak >= 80
            && declineFraction >= 1.00
            && peakFraction >= 0.19
            && observedWidthFraction >= 0.10
            && observedWidthFraction <= 0.40
            && localPeakRate <= 0.25
            && startNormalized <= 0.45;
    }
    return (enough && distanceAfterPeak >= 32) || distanceAfterPeak >= 176;
}

[[maybe_unused]] bool CrediblePeak(
    const std::vector<Value>& values,
    Value bestValue)
{
    if (values.size() < 6) {
        return false;
    }
    const auto [baseline, noise] = Levels(values);
    const double prominence = static_cast<double>(bestValue) - baseline;
    return prominence >= 8000.0
        && prominence >= 2.50 * std::max(1.0, baseline)
        && prominence >= 15.0 * noise;
}

bool QuadraticVertex(
    Position x0,
    Value y0,
    Position x1,
    Value y1,
    Position x2,
    Value y2,
    double& vertex)
{
    if (x1 <= x0 || x2 <= x1) {
        return false;
    }
    const double firstSlope =
        static_cast<double>(y1 - y0) / static_cast<double>(x1 - x0);
    const double secondSlope =
        static_cast<double>(y2 - y1) / static_cast<double>(x2 - x1);
    const double curvature =
        (secondSlope - firstSlope) / static_cast<double>(x2 - x0);
    if (curvature >= 0.0) {
        return false;
    }
    const double linear = firstSlope - curvature * static_cast<double>(x0 + x1);
    vertex = -linear / (2.0 * curvature);
    return true;
}

Position PythonRound(double value)
{
    const double lower = std::floor(value);
    const double fraction = value - lower;
    if (fraction < 0.5) {
        return static_cast<Position>(lower);
    }
    if (fraction > 0.5) {
        return static_cast<Position>(lower + 1.0);
    }
    const Position lowerInteger = static_cast<Position>(lower);
    return lowerInteger % 2 == 0 ? lowerInteger : lowerInteger + 1;
}

[[maybe_unused]] std::vector<std::pair<Position, Position>> ModelIntervals(
    const std::vector<Position>& positions,
    const std::vector<Value>& values,
    Position middleTwice)
{
    const size_t count = values.size();
    std::vector<Value> ordered = values;
    std::sort(ordered.begin(), ordered.end());
    const size_t baselineIndex = count / 5 == 0 ? 0 : count / 5 - 1;
    const double baseline = static_cast<double>(ordered[baselineIndex]);
    const Value bestValue = *std::max_element(values.begin(), values.end());
    const double cutoff = baseline + 0.93 * (static_cast<double>(bestValue) - baseline);

    std::vector<size_t> candidates;
    for (size_t index = 0; index < count; ++index) {
        const Value left = index == 0 ? values[index] : values[index - 1];
        const Value right = index + 1 < count ? values[index + 1] : values[index];
        if (
            values[index] >= left
            && values[index] >= right
            && static_cast<double>(values[index]) >= cutoff) {
            candidates.push_back(index);
        }
    }
    if (candidates.empty()) {
        candidates.push_back(static_cast<size_t>(
            std::max_element(values.begin(), values.end()) - values.begin()));
    }
    std::sort(candidates.begin(), candidates.end(), [&](size_t left, size_t right) {
        if (values[left] != values[right]) {
            return values[left] > values[right];
        }
        const Position leftDistance = std::llabs(2 * positions[left] - middleTwice);
        const Position rightDistance = std::llabs(2 * positions[right] - middleTwice);
        if (leftDistance != rightDistance) {
            return leftDistance < rightDistance;
        }
        return positions[left] < positions[right];
    });

    std::vector<std::pair<Position, Position>> intervals;
    const size_t candidateCount = std::min<size_t>(3, candidates.size());
    intervals.reserve(candidateCount);
    for (size_t ordinal = 0; ordinal < candidateCount; ++ordinal) {
        const size_t index = candidates[ordinal];
        double centre = static_cast<double>(positions[index]);
        if (index > 0 && index + 1 < count) {
            double vertex = 0.0;
            if (QuadraticVertex(
                    positions[index - 1], values[index - 1],
                    positions[index], values[index],
                    positions[index + 1], values[index + 1], vertex)) {
                const Position leftLimit = positions[index - 1] - kModelRadius;
                const Position rightLimit = positions[index + 1] + kModelRadius;
                if (
                    static_cast<double>(leftLimit) <= vertex
                    && vertex <= static_cast<double>(rightLimit)) {
                    centre = vertex;
                }
            }
        }
        const Position centreIndex = PythonRound(centre);
        intervals.emplace_back(
            std::max<Position>(0, centreIndex - kModelRadius),
            std::min(positions.back(), centreIndex + kModelRadius));
    }
    return intervals;
}

std::vector<std::pair<Position, Position>> CandidateIntervals(
    const std::vector<Position>& positions,
    const std::vector<Value>& values,
    Position middleTwice)
{
    const size_t count = values.size();
    std::vector<Value> ordered = values;
    std::sort(ordered.begin(), ordered.end());
    const size_t baselineIndex = count / 5 == 0 ? 0 : count / 5 - 1;
    const double baseline = static_cast<double>(ordered[baselineIndex]);
    const Value bestValue = *std::max_element(values.begin(), values.end());
    const double cutoff = baseline + 0.93 * (static_cast<double>(bestValue) - baseline);

    std::vector<size_t> candidates;
    for (size_t index = 0; index < count; ++index) {
        const Value left = index == 0 ? values[index] : values[index - 1];
        const Value right = index + 1 < count ? values[index + 1] : values[index];
        if (
            values[index] >= left
            && values[index] >= right
            && static_cast<double>(values[index]) >= cutoff) {
            candidates.push_back(index);
        }
    }
    if (candidates.empty()) {
        candidates.push_back(static_cast<size_t>(
            std::max_element(values.begin(), values.end()) - values.begin()));
    }
    std::sort(candidates.begin(), candidates.end(), [&](size_t left, size_t right) {
        if (values[left] != values[right]) {
            return values[left] > values[right];
        }
        const Position leftDistance = std::llabs(2 * positions[left] - middleTwice);
        const Position rightDistance = std::llabs(2 * positions[right] - middleTwice);
        if (leftDistance != rightDistance) {
            return leftDistance < rightDistance;
        }
        return positions[left] < positions[right];
    });

    std::vector<std::pair<Position, Position>> intervals;
    const size_t candidateCount = std::min<size_t>(3, candidates.size());
    intervals.reserve(candidateCount);
    for (size_t rank = 0; rank < candidateCount; ++rank) {
        const size_t index = candidates[rank];
        const size_t reach = rank == 0 ? 2 : 1;
        const size_t leftSample = index > reach ? index - reach : 0;
        const size_t rightSample = std::min(count - 1, index + reach);
        const Position left = leftSample == 0 ? 0 : positions[leftSample] + 1;
        const Position right = rightSample == count - 1
            ? positions.back()
            : positions[rightSample] - 1;
        intervals.emplace_back(left, right);
    }
    return intervals;
}

struct EntryGap {
    Position left = 0;
    Position right = -1;
    Position entry = 0;
    bool present = false;
};

}  // namespace

uint32_t FindPeak(uint32_t arrayLength)
{
    if (arrayLength == 0) {
        return 1;
    }

    std::vector<Position> positions;
    std::vector<Value> values;
    std::unordered_set<Position> measured;
    int queryCount = 0;
    Position currentPosition = 0;
    const Position length = static_cast<Position>(arrayLength);
    const Position middleTwice = length - 1;
    Position bestIndex = 0;
    Value bestValue = -1;
    bool dense = false;
    bool narrowFinished = false;
    Position denseLeftLimit = 0;
    Position denseEntry = -1;
    Value denseEntryValue = 0;

    const auto isBetter = [&](Position index, Value value) {
        if (value != bestValue) {
            return value > bestValue;
        }
        return std::make_pair(std::llabs(2 * index - middleTwice), index)
            < std::make_pair(std::llabs(2 * bestIndex - middleTwice), bestIndex);
    };

    const auto query = [&](Position index) {
        const Value value = static_cast<Value>(RunMotor(static_cast<uint32_t>(index)));
        ++queryCount;
        currentPosition = index;
        measured.insert(index);
        if (isBetter(index, value)) {
            bestIndex = index;
            bestValue = value;
        }
        return value;
    };

    positions.push_back(0);
    values.push_back(query(0));
    while (currentPosition < length - 1 && queryCount < kMaxQueries - 1) {
        Position step = dense ? 1 : ScanStep(currentPosition, values);
        if (!dense && currentPosition >= 128) {
            const int remainingBudget = std::max(
                1, kMaxQueries - queryCount - 32);
            const Position budgetStep = static_cast<Position>(std::ceil(
                static_cast<double>(length - 1 - currentPosition)
                / static_cast<double>(remainingBudget)));
            step = std::max(step, budgetStep);
        }
        const Position nextPosition = std::min(length - 1, currentPosition + step);
        positions.push_back(nextPosition);
        values.push_back(query(nextPosition));

        if (!dense && EnterDense(positions, values)) {
            denseLeftLimit = positions[positions.size() - 2] + 1;
            denseEntry = positions.back();
            denseEntryValue = values.back();
            dense = true;
        } else if (!dense && NarrowProbeTrigger(positions, values)) {
            const size_t probeStart = values.size() - 1;
            const std::vector<Value> preProbe(values.begin(), values.begin() + probeStart);
            const auto [probeBaseline, probeNoise] = Levels(preProbe);

            for (int probe = 0; probe < 16; ++probe) {
                if (currentPosition >= length - 1 || queryCount >= kMaxQueries - 1) {
                    break;
                }
                const Position probePosition = currentPosition + 1;
                positions.push_back(probePosition);
                values.push_back(query(probePosition));

                const auto segmentBegin = values.begin() + static_cast<std::ptrdiff_t>(probeStart);
                const auto peakIterator = std::max_element(segmentBegin, values.end());
                const Value segmentPeak = *peakIterator;
                const size_t segmentPeakOffset =
                    static_cast<size_t>(peakIterator - segmentBegin);
                const size_t segmentSize = values.size() - probeStart;
                const double segmentProminence =
                    static_cast<double>(segmentPeak) - probeBaseline;
                if (
                    segmentSize - 1 - segmentPeakOffset >= 2
                    && segmentProminence > 0.0
                    && static_cast<double>(values.back())
                        <= probeBaseline + 0.08 * segmentProminence) {
                    break;
                }
            }

            const auto segmentBegin = values.begin() + static_cast<std::ptrdiff_t>(probeStart);
            const auto peakIterator = std::max_element(segmentBegin, values.end());
            const Value segmentPeak = *peakIterator;
            const size_t segmentPeakOffset = static_cast<size_t>(peakIterator - segmentBegin);
            const size_t segmentSize = values.size() - probeStart;
            const double segmentProminence = static_cast<double>(segmentPeak) - probeBaseline;
            int supported = 0;
            for (auto iterator = segmentBegin; iterator != values.end(); ++iterator) {
                supported += static_cast<double>(*iterator)
                        >= probeBaseline + 0.08 * segmentProminence
                    ? 1
                    : 0;
            }
            narrowFinished =
                segmentProminence >= 18.0 * probeNoise
                && segmentProminence >= 1.10 * std::max(1.0, probeBaseline)
                && supported >= 3
                && segmentSize - 1 - segmentPeakOffset >= 2
                && static_cast<double>(values.back())
                    <= probeBaseline + 0.12 * segmentProminence;
            if (narrowFinished) {
                break;
            }
            continue;
        }

        if (nextPosition == length - 1) {
            break;
        }
        if (dense) {
            if (DensePeakFinished(values, positions, bestIndex, bestValue)) {
                const bool almostTied =
                    bestIndex - denseEntry <= 3
                    && static_cast<double>(bestValue - denseEntryValue)
                        <= 0.001 * static_cast<double>(
                            std::max<Value>(1, std::llabs(bestValue)));
                if (bestIndex <= denseEntry || almostTied) {
                    for (
                        Position index = denseEntry - 1;
                        index >= denseLeftLimit && queryCount < kMaxQueries - 1;
                        --index) {
                        if (measured.find(index) == measured.end()) {
                            query(index);
                        }
                    }
                }
                break;
            }
        } else if (ShouldTurn(length, positions, values)) {
            break;
        }
    }

    if (!dense && !narrowFinished) {
        const auto intervals = CandidateIntervals(positions, values, middleTwice);
        std::set<Position> selected;
        int available = kMaxQueries - queryCount - 1;
        for (const auto& [left, right] : intervals) {
            std::vector<Position> additions;
            for (Position index = left; index <= right; ++index) {
                if (
                    measured.find(index) == measured.end()
                    && selected.find(index) == selected.end()) {
                    additions.push_back(index);
                }
            }
            if (static_cast<int>(additions.size()) <= available) {
                selected.insert(additions.begin(), additions.end());
                available -= static_cast<int>(additions.size());
            }
        }
        for (auto iterator = selected.rbegin(); iterator != selected.rend(); ++iterator) {
            query(*iterator);
        }
    }

    if (currentPosition != bestIndex && queryCount < kMaxQueries) {
        query(bestIndex);
    }
    return 0;
}
