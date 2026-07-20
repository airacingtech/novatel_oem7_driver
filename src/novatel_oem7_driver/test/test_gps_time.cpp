#include <gtest/gtest.h>

#include "gps_time.hpp"

// Reference from a live PwrPak7 TIMEA log: GPS week 2428, 74693.000 s into the
// week == 2026-07-19 20:44:35 UTC == unix 1784493875.
TEST(GpsTime, MatchesReceiverTimeLog)
{
  EXPECT_EQ(
    art::gps_week_ms_to_unix_ns(2428, 74693000),
    1784493875LL * art::kNsPerSec);
}

TEST(GpsTime, WeekTowNsAgreesWithWeekMs)
{
  EXPECT_EQ(
    art::gps_week_tow_ns_to_unix_ns(2428, 74693000LL * 1000000),
    art::gps_week_ms_to_unix_ns(2428, 74693000));
}

TEST(GpsTime, PreservesNanosecondResolution)
{
  EXPECT_EQ(art::gps_ns_to_unix_ns(1) - art::gps_ns_to_unix_ns(0), 1);
  EXPECT_EQ(
    art::gps_week_ms_to_unix_ns(2428, 1) - art::gps_week_ms_to_unix_ns(2428, 0),
    1000000);
  EXPECT_EQ(
    art::gps_week_tow_ns_to_unix_ns(2428, 1) - art::gps_week_tow_ns_to_unix_ns(2428, 0),
    1);
}
