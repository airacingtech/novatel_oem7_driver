#ifndef __GPS_TIME_HPP__
#define __GPS_TIME_HPP__

#include <cstdint>

namespace art
{

/// GPS -> Unix (UTC) time conversion for GNSS-disciplined sensor stamping.
///
/// GNSS receivers report measurement time on the GPS timescale (epoch
/// 1980-01-06, no leap seconds), disciplined to true GPS time within ~ns.
/// Stamping directly from it puts every GNSS-disciplined sensor on one common
/// time base with no host-side estimation. The fixed 18 s leap offset is
/// current since 2017; update if IERS announces another leap second (the
/// receivers themselves report it, e.g. the OEM7 TIME log).
constexpr int64_t kGpsEpochUnixS = 315964800;  ///< 1980-01-06 00:00:00 UTC
constexpr int64_t kGpsMinusUtcS = 18;
constexpr int64_t kSecPerWeek = 604800;
constexpr int64_t kNsPerSec = 1000000000;

/// Absolute GPS time (ns since GPS epoch) -> Unix UTC ns.
constexpr int64_t gps_ns_to_unix_ns(int64_t gps_ns)
{
  return gps_ns + (kGpsEpochUnixS - kGpsMinusUtcS) * kNsPerSec;
}

/// GPS week + milliseconds into week (OEM7 header / SBF WNc+TOW) -> Unix UTC ns.
constexpr int64_t gps_week_ms_to_unix_ns(uint32_t week, uint32_t week_ms)
{
  return gps_ns_to_unix_ns(
    static_cast<int64_t>(week) * kSecPerWeek * kNsPerSec +
    static_cast<int64_t>(week_ms) * 1000000);
}

/// GPS week + nanoseconds into week (VectorNav GpsTow) -> Unix UTC ns.
constexpr int64_t gps_week_tow_ns_to_unix_ns(uint32_t week, int64_t tow_ns)
{
  return gps_ns_to_unix_ns(
    static_cast<int64_t>(week) * kSecPerWeek * kNsPerSec + tow_ns);
}

}  // namespace art

#endif  // __GPS_TIME_HPP__
