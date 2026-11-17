# FocusHunter

FocusHunter is a C++17 autofocus solver that searches for the sharpest lens position under a budget for motor queries, travel, direction reversals, and overshoot. It was developed for the Light Chaser optical track at the [Huawei 2026 France Tech Arena](https://huawei.agorize.com/en/challenges/2026-france-tech-arena).

## The problem

The motor starts at position zero. Each call to `RunMotor(index)` moves the lens to a zero-based position and returns one sharpness reading. `FindPeak(length)` uses those readings to direct the search and leaves the motor parked at its selected position. Noise, narrow peaks, and competing peaks make the stopping decision as important as the search itself.

The challenge score starts at 3,000 points per case. The penalties are 200 points per position of final error, one per query, one per position traveled, 30 per direction reversal, and one per position of overshoot. A case scores zero beyond three positions of final error or at 200 queries, seven reversals, or 200 positions of overshoot.

## Search strategy

1. **Scan forward with adaptive steps.** Estimate the background sharpness and noise from the readings so far. Use wider steps in low-signal regions and tighten the spacing on a promising rise.
2. **Look for a supported peak.** Switch to consecutive readings near a credible rise or run a short probe for a narrow peak. The stopping rules use the peak’s prominence, width, and the decline after it to reduce the influence of isolated spikes.
3. **Refine and park.** When further refinement is needed, inspect a small set of candidate intervals in descending position order to limit reversals. Reserve a query to return to the sharpest position measured and cap the search at 199 calls.

The search is heuristic. A stronger peak beyond the observed region can be missed when the evidence favors stopping early.

## Run the demo

The standalone demo generates a deterministic sharpness curve and simulates the motor in memory. It requires a C++17 compiler and the standard library.

```sh
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror demo.cpp autofocus_cpp/solution.cpp -o focus-hunter
./focus-hunter
```

On Windows with MinGW and PowerShell, use `-o focus-hunter.exe` and run `.\focus-hunter.exe`.

Expected output:

```text
Target index: 172
Final index: 172
Motor queries: 77
Motor travel: 204
Direction reversals: 1
Overshoot: 16
Simulated score: 2673 / 3000
```

## Evaluation

| Evaluation | Version | Result |
| --- | --- | --- |
| Included synthetic demo | Published solver | 2,673 / 3,000, exact peak in 77 queries |
| Organizer’s public sample, run locally | Published solver | 2,832 / 3,000, exact peak in 32 queries |
| Competition judge | Earlier generic version | 1,820.5 average, as recorded during the contest |

The organizer’s sample and the synthetic demo each measure a single curve. The judge score comes from a separate evaluation of the earlier version. The published solver adds a stopping rule for low-prominence peaks that meet stricter shape checks.

## Repository layout

- [`autofocus_cpp/solution.cpp`](autofocus_cpp/solution.cpp) implements `FindPeak` and the search policy.
- [`autofocus_cpp/solution.h`](autofocus_cpp/solution.h) declares the solver entry point.
- [`demo.cpp`](demo.cpp) supplies the simulated `RunMotor` callback and reports the result.
