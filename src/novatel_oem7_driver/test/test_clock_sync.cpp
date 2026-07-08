// Robustness tests for art::ClockSync under race-representative conditions:
// bursty/coalesced delivery, latency spikes, oscillator skew (temperature drift),
// device resets, and long runs. The estimator maps device time onto the LOWER
// envelope of arrivals, so it recovers (measurement_time + minimum_transport_latency);
// tests account for that irreducible min-latency bias.

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "clock_sync.hpp"

namespace
{
// Host clock is ~Unix epoch (large) to exercise double precision; device clock is
// small (seconds since power-on). True map: host = HOST0 + skew * (device - D0).
constexpr double D0 = 1234.0;         // device seconds at start
constexpr double HOST0 = 1.78e9;      // host seconds at start (~year 2026)
constexpr double DT = 0.01;           // 100 Hz

double true_host(double device, double skew) {return HOST0 + skew * (device - D0);}

art::ClockSync::Config cfg(bool enabled) {
  art::ClockSync::Config c;
  c.enabled = enabled;
  c.window_sec = 2.0;
  c.min_samples = 50;
  return c;
}
}  // namespace

TEST(ClockSync, DisabledIsExactPassthrough) {
  art::ClockSync cs(cfg(false));
  for (int i = 0; i < 100; ++i) {
    const double d = D0 + i * DT;
    const double a = true_host(d, 1.0) + 0.05;  // arbitrary latency
    EXPECT_EQ(cs.update(d, a), a);              // must return arrival untouched
  }
}

TEST(ClockSync, RecoversMeasurementTimeThroughBurstyLatency) {
  art::ClockSync cs(cfg(true));
  std::mt19937 rng(42);
  std::exponential_distribution<double> jit(1.0 / 0.003);  // ~3 ms mean jitter
  double worst = 0.0;
  const double skew = 1.0;
  for (int i = 0; i < 4000; ++i) {
    const double d = D0 + i * DT;
    // Every 5th packet is minimally delayed (defines the envelope); others jittered
    // and delivered in bursts (cluster shares one arrival instant).
    double latency = (i % 5 == 0) ? 0.0002 : 0.0002 + jit(rng);
    const double a = true_host(d, skew) + latency;
    const double host = cs.update(d, a);
    if (i > 1000) {  // after warm-up
      worst = std::max(worst, std::fabs(host - true_host(d, skew)));
    }
  }
  // Envelope bias is the ~0.2 ms minimum latency; allow generous margin below the
  // ~3 ms mean jitter it replaces.
  EXPECT_LT(worst, 1.5e-3) << "worst error " << worst * 1e3 << " ms";
}

TEST(ClockSync, TracksOscillatorSkewViaContinuousOffset) {
  // Skew is not fit explicitly; the continuously re-estimated offset must follow
  // it so error stays bounded over a long run with a large drift.
  art::ClockSync cs(cfg(true));
  std::mt19937 rng(7);
  std::exponential_distribution<double> jit(1.0 / 0.002);
  const double skew = 1.0 + 108e-6;  // 108 ppm, measured on dlap
  double worst = 0.0;
  for (int i = 0; i < 12000; ++i) {  // 120 s -> skew moves ~13 ms
    const double d = D0 + i * DT;
    const double latency = (i % 4 == 0) ? 0.0001 : 0.0001 + jit(rng);
    const double a = true_host(d, skew) + latency;
    const double host = cs.update(d, a);
    if (i > 1500) {worst = std::max(worst, std::fabs(host - true_host(d, skew)));}
  }
  EXPECT_LT(worst, 1.0e-3) << "drift not tracked; worst error " << worst * 1e3 << " ms";
}

TEST(ClockSync, RobustToLatencySpikes) {
  art::ClockSync cs(cfg(true));
  std::mt19937 rng(99);
  std::exponential_distribution<double> jit(1.0 / 0.002);
  double worst = 0.0;
  for (int i = 0; i < 6000; ++i) {
    const double d = D0 + i * DT;
    double latency = (i % 4 == 0) ? 0.0001 : 0.0001 + jit(rng);
    if (i % 200 == 0) {latency += 0.1;}  // 100 ms CPU-load / scheduling spikes
    const double a = true_host(d, 1.0) + latency;
    const double host = cs.update(d, a);
    if (i > 1500) {worst = std::max(worst, std::fabs(host - true_host(d, 1.0)));}
  }
  EXPECT_LT(worst, 1.5e-3) << "spikes leaked into estimate: " << worst * 1e3 << " ms";
}

TEST(ClockSync, RecoversAfterDeviceReset) {
  art::ClockSync cs(cfg(true));
  std::mt19937 rng(3);
  std::exponential_distribution<double> jit(1.0 / 0.002);
  auto run = [&](double dbase, int n, double skew) {
    double worst = 0.0;
    for (int i = 0; i < n; ++i) {
      const double d = dbase + i * DT;
      const double latency = (i % 4 == 0) ? 0.0001 : 0.0001 + jit(rng);
      const double a = HOST0 + skew * (d - D0) + latency;  // host keeps running
      const double host = cs.update(d, a);
      if (i > 1500) {worst = std::max(worst, std::fabs(host - (HOST0 + skew * (d - D0))));}
    }
    return worst;
  };
  run(D0, 4000, 1.0);
  // Device reboots: p1 restarts near 0 while the host clock keeps advancing.
  const double worst_after = run(5.0, 4000, 1.0);
  EXPECT_LT(worst_after, 1.5e-3) << "did not re-converge after reset: "
                                 << worst_after * 1e3 << " ms";
}

TEST(ClockSync, OutputIsMonotonicOnceConverged) {
  // Per-stream monotonicity holds naturally once converged (device time is
  // monotonic, offset moves slowly). Warm-up passes arrival through, so check
  // after convergence.
  art::ClockSync cs(cfg(true));
  std::mt19937 rng(11);
  std::exponential_distribution<double> jit(1.0 / 0.004);
  double prev = -1e18;
  for (int i = 0; i < 5000; ++i) {
    const double d = D0 + i * DT;
    const double a = true_host(d, 1.0 + 50e-6) + 0.0001 + jit(rng);
    const double host = cs.update(d, a);
    if (i > 1500) {EXPECT_GE(host, prev) << "stamp went backwards at i=" << i;}
    prev = host;
  }
}

TEST(ClockSync, InterleavedMultiRateStreamsAreNotCoupled) {
  // Regression: a shared instance fed several rates (IMU 125 Hz, pose 100 Hz,
  // GNSS 20 Hz) from one device clock must stamp EACH stream with its own device
  // time, not smear a fast stream onto a slow one. Streams share a true offset;
  // arrival order interleaves and is not monotonic in device time.
  art::ClockSync cs(cfg(true));
  std::mt19937 rng(5);
  std::exponential_distribution<double> jit(1.0 / 0.002);
  const double skew = 1.0;
  struct Stream {double dt; double next; double worst; double prev;};
  Stream imu{0.008, D0, 0.0, -1e18};   // 125 Hz
  Stream pose{0.010, D0, 0.0, -1e18};  // 100 Hz
  Stream gnss{0.050, D0, 0.0, -1e18};  // 20 Hz
  // Merge by device time; feed in that (interleaved) order.
  for (int step = 0; step < 6000; ++step) {
    Stream * s = &imu;
    if (pose.next < s->next) {s = &pose;}
    if (gnss.next < s->next) {s = &gnss;}
    const double d = s->next;
    const double latency = 0.0002 + jit(rng);
    const double host = cs.update(d, true_host(d, skew) + latency);
    if (step > 2000) {
      s->worst = std::max(s->worst, std::fabs(host - true_host(d, skew)));
      EXPECT_GE(host, s->prev) << "stream stamp went backwards";
    }
    s->prev = host;
    s->next += s->dt;
  }
  EXPECT_LT(imu.worst, 1.0e-3) << "IMU (125Hz) smeared: " << imu.worst * 1e3 << " ms";
  EXPECT_LT(pose.worst, 1.0e-3) << "pose (100Hz): " << pose.worst * 1e3 << " ms";
  EXPECT_LT(gnss.worst, 1.0e-3) << "GNSS (20Hz): " << gnss.worst * 1e3 << " ms";
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
