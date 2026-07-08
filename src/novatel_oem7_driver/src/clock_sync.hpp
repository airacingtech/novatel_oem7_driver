#ifndef FUSION_ENGINE_DRIVER__UTILS__CLOCK_SYNC_HPP_
#define FUSION_ENGINE_DRIVER__UTILS__CLOCK_SYNC_HPP_

#include <algorithm>
#include <cstddef>
#include <deque>
#include <vector>

namespace art
{

/// Robust device-clock -> host-clock estimator for network sensor drivers.
///
/// A sensor delivers each measurement with its own device clock time `d`; the
/// host observes it at arrival time `a = host(d) + latency`, latency >= 0. Bursty
/// transport, packet coalescing and CPU-load spikes only ever push `a` ABOVE the
/// true line, so the minimum-latency samples define host(d).
///
/// It tracks the offset `host - device` as a low quantile of `(a - d)` over a
/// short sliding window, smoothed with a rate-independent time constant, and maps
/// `host(d) = d + offset`. Oscillator skew (the device and host clocks running at
/// slightly different rates) shows up as a slowly drifting offset, which the
/// continuous re-estimation follows; the residual lag is skew x window (sub-ms).
/// We deliberately do NOT fit an explicit slope: over the few-second baseline a
/// driver can hold, the ~1 ms skew signal is buried under several ms of latency
/// jitter, so a slope fit is unstable and injects more error than it removes.
///
/// ROS-free and header-only so it can be unit-tested with synthetic adversarial
/// inputs and dropped into any driver (fusion_engine / novatel / vectornav).
/// Single-threaded: feed it from the one thread that reads the socket.
class ClockSync
{
public:
  struct Config
  {
    bool enabled = false;             ///< off => passthrough (host stamp == arrival)
    double window_sec = 2.0;          ///< sliding window the offset is estimated over
    std::size_t min_samples = 50;     ///< samples before the estimate is trusted
    double reset_gap_sec = 1.0;       ///< backward device-time jump => device reset
    double smoothing_tau_sec = 1.0;   ///< time constant for offset smoothing
    double envelope_quantile = 0.05;  ///< low quantile of (a-d) used as the offset
  };

  explicit ClockSync(const Config & cfg)
  : cfg_(cfg) {}

  bool enabled() const {return cfg_.enabled;}
  bool converged() const {return converged_;}
  double offset() const {return offset_;}

  /// Feed one (device seconds, host arrival seconds) pair; return the host-clock
  /// stamp to publish: device time + estimated offset. Returns `arrival_s`
  /// unchanged while disabled or warming up.
  ///
  /// This is a pure per-call offset mapper with NO output-sequence state, so it
  /// can be shared across several interleaved message streams (e.g. IMU 125 Hz,
  /// pose 100 Hz, GNSS 20 Hz) fed from one device clock. Monotonicity is a
  /// per-stream property and holds naturally: each stream's device time is
  /// monotonic and the offset moves slowly, so `device + offset` is monotonic
  /// within a stream. A shared output clamp would instead couple the streams and
  /// smear the faster ones onto the slower one's cadence.
  double update(double device_s, double arrival_s)
  {
    if (!cfg_.enabled) {return arrival_s;}

    if (have_last_ && device_s < last_device_s_ - cfg_.reset_gap_sec) {
      reset();  // device reboot / clock reset -> start over
    }
    have_last_ = true;
    last_device_s_ = device_s;

    samples_.push_back(Sample{device_s, arrival_s});
    while (samples_.size() > 2 &&
      device_s - samples_.front().d > cfg_.window_sec)
    {
      samples_.pop_front();
    }

    fit(arrival_s);

    return converged_ ? device_s + offset_ : arrival_s;
  }

private:
  struct Sample
  {
    double d;
    double a;
  };

  void reset()
  {
    samples_.clear();
    converged_ = false;
    offset_ = 0.0;
  }

  /// Offset = a low quantile of (arrival - device) over the window (a stable
  /// stand-in for "minimum latency" that does not jump when a single lowest
  /// sample ages out), smoothed with a rate-independent time constant.
  void fit(double arrival_s)
  {
    const std::size_t n = samples_.size();
    if (n < cfg_.min_samples) {converged_ = false; return;}

    std::vector<double> off(n);
    for (std::size_t i = 0; i < n; ++i) {
      off[i] = samples_[i].a - samples_[i].d;
    }
    std::size_t k = static_cast<std::size_t>(cfg_.envelope_quantile * n);
    if (k >= n) {k = n - 1;}
    std::nth_element(off.begin(), off.begin() + k, off.end());
    const double offset_raw = off[k];

    if (!converged_) {
      offset_ = offset_raw;
      converged_ = true;
    } else {
      const double dt = arrival_s - last_fit_arrival_;
      const double al = (dt > 0.0) ? dt / (cfg_.smoothing_tau_sec + dt) : 0.0;
      offset_ += al * (offset_raw - offset_);
    }
    last_fit_arrival_ = arrival_s;
  }

  Config cfg_;
  std::deque<Sample> samples_;
  double offset_ = 0.0;
  double last_device_s_ = 0.0;
  double last_fit_arrival_ = 0.0;
  bool have_last_ = false;
  bool converged_ = false;
};

}  // namespace art

#endif  // FUSION_ENGINE_DRIVER__UTILS__CLOCK_SYNC_HPP_
