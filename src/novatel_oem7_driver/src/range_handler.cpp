////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////


#include <memory>

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>


#include <driver_parameter.hpp>
#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include <oem7_ros_publisher.hpp>

#include "novatel_oem7_msgs/msg/range.hpp"

using novatel_oem7_msgs::msg::RANGE;


namespace novatel_oem7_driver
{
  /***
   * Handles Range
   */
  class RangeHandler: public Oem7MessageHandlerIf
  {
  private:
    rclcpp::Node* node_;
  
    std::unique_ptr<Oem7RosPublisher<RANGE>>       range_pub;
    std::unique_ptr<Oem7RosPublisher<RANGE>>       range_1_pub;

    std::shared_ptr<RANGE>   range_holder_;

    void publishRangeMessage()
    {
      if (range_holder_->nov_header.message_type == 0)
      {
        range_pub->publish(range_holder_);
      }
      else
      {
        range_1_pub->publish(range_holder_);
      }
    }


  public:
    RangeHandler()
    {
    }

    ~RangeHandler()
    {
    }

    void initialize(rclcpp::Node& node)
    {
      node_ = &node;

      range_pub  = std::make_unique<Oem7RosPublisher<RANGE>>( "RANGE",  node);
      range_1_pub  = std::make_unique<Oem7RosPublisher<RANGE>>( "RANGE_1",  node);
    }

    const MessageIdRecords& getMessageIds()
    {
      static const MessageIdRecords MSG_IDS(
                                      {
                                        {RANGE_OEM7_MSGID,            MSGFLAG_NONE},
                                      }
                                    );
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      if(msg->getMessageId()== RANGE_OEM7_MSGID)
      {
        MakeROSMessage(msg, range_holder_);
        publishRangeMessage();
      }
      else
      {
        assert(false);
      }
    }
};

} // namespace novatel_oem7_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::RangeHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
