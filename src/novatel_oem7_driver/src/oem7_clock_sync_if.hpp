#ifndef __OEM7_CLOCK_SYNC_IF_HPP__
#define __OEM7_CLOCK_SYNC_IF_HPP__

#include <rclcpp/rclcpp.hpp>

namespace novatel_oem7_driver
{

/**
 * Implemented by the message node so Oem7RosPublisher can stamp with a
 * device-clock-synced time without depending on the concrete node type.
 * The node computes the synced stamp once per raw message (from its GPS time)
 * before dispatching to handlers; publishers read it here.
 */
class ClockSyncedNodeIf
{
public:
  virtual ~ClockSyncedNodeIf() = default;

  /** @return true when device-time stamping is enabled and converged. */
  virtual bool clockSyncEnabled() const = 0;

  /** @return host-clock stamp for the message currently being handled. */
  virtual rclcpp::Time syncedStamp() const = 0;

  virtual bool gpsTimeFine() const = 0;
};

}  // namespace novatel_oem7_driver

#endif  // __OEM7_CLOCK_SYNC_IF_HPP__
