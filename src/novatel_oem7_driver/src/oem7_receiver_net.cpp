////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
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

#include <novatel_oem7_driver/oem7_receiver_if.hpp>
#include <oem7_receiver.hpp>


#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <driver_parameter.hpp>

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace novatel_oem7_driver
{

  // Dedicated reader thread decouples socket recv from decode+publish.
  // The single-threaded decode/publish pipeline can block (DDS write, handler work);
  // during those stalls a synchronous recv loop would let the kernel socket buffer
  // fill and eventually drop. The reader thread drains the socket continuously and
  // hands chunks to the decode loop via a bounded queue.
  template <class T>
  class Oem7ReceiverNet: public Oem7Receiver<typename T::socket>
  {
    using Oem7Receiver<typename T::socket>::node_;

    static constexpr size_t MAX_QUEUE_ENTRIES = 8192;
    static constexpr size_t RX_CHUNK_BYTES    = 65536;

    std::thread                             reader_thread_;
    std::atomic<bool>                       reader_stop_{false};
    std::mutex                              queue_mtx_;
    std::condition_variable                 queue_cv_;
    std::deque<std::vector<uint8_t>>        rx_queue_;
    std::vector<uint8_t>                    pending_;
    size_t                                  pending_off_{0};
    boost::system::error_code               async_err_;
    uint64_t                                dropped_chunks_{0};

    void reader_loop()
    {
      std::vector<uint8_t> buf(RX_CHUNK_BYTES);
      while(!reader_stop_.load(std::memory_order_relaxed))
      {
        boost::system::error_code err;
        boost::array<boost::asio::mutable_buffer, 1> bufs = {
          boost::asio::buffer(buf.data(), buf.size())
        };
        size_t n = this->endpoint_.receive(bufs, 0, err);

        if(err)
        {
          std::lock_guard<std::mutex> lk(queue_mtx_);
          async_err_ = err;
          queue_cv_.notify_all();
          return;
        }
        if(n == 0)
        {
          continue;
        }

        std::lock_guard<std::mutex> lk(queue_mtx_);
        if(rx_queue_.size() >= MAX_QUEUE_ENTRIES)
        {
          rx_queue_.pop_front();
          ++dropped_chunks_;
          if((dropped_chunks_ % 100) == 1)
          {
            RCLCPP_WARN_STREAM(node_->get_logger(),
              "Oem7ReceiverNet: reader queue overflow; total dropped chunks=" << dropped_chunks_);
          }
        }
        rx_queue_.emplace_back(buf.begin(), buf.begin() + n);
        queue_cv_.notify_one();
      }
    }

    void reset_reader_state()
    {
      std::lock_guard<std::mutex> lk(queue_mtx_);
      rx_queue_.clear();
      pending_.clear();
      pending_off_ = 0;
      async_err_.clear();
    }

    void join_reader_if_any()
    {
      if(reader_thread_.joinable())
      {
        reader_stop_.store(true);
        boost::system::error_code err;
        this->endpoint_.close(err);  // Unblock any in-flight receive().
        queue_cv_.notify_all();
        reader_thread_.join();
      }
      reader_stop_.store(false);
    }

    void endpoint_try_open()
    {
      if(this->endpoint_.is_open())
      {
        return;
      }

      // Socket went down (or first open). Make sure any prior reader is joined
      // and queue state is cleared before reopening.
      join_reader_if_any();
      reset_reader_state();

      static DriverParameter<std::string> recvr_ip_addr("oem7_ip_addr", "", *node_);
      static DriverParameter<int>         recvr_port(   "oem7_port",    0,  *node_);

      RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Oem7Net " << (T::v4().protocol() == IPPROTO_TCP ? "TCP" : "UDP") <<
                      "['" << recvr_ip_addr.value() << "' : " << recvr_port.value() << "]");

      boost::system::error_code err;

      this->endpoint_.close(err); // Doesn't matter if we fail.
      this->endpoint_.connect(typename T::endpoint(boost::asio::ip::address::from_string(recvr_ip_addr.value()), recvr_port.value()), err);
      // Proceed regardless; successful connection does not guarantee subsequent operations will succeed.

      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "Oem7Net socket open: '" << this->endpoint_.is_open() << "; OS error= " << err.value());

      static const std::string CONN_PRIMER("\r\n");
      endpoint_write(boost::asio::buffer(CONN_PRIMER), err);

      if(this->endpoint_.is_open())
      {
        reader_stop_.store(false);
        reader_thread_ = std::thread(&Oem7ReceiverNet::reader_loop, this);
      }
    }

    virtual size_t endpoint_read(boost::asio::mutable_buffer buf, boost::system::error_code& err)
    {
      std::unique_lock<std::mutex> lk(queue_mtx_);

      if(pending_off_ >= pending_.size())
      {
        queue_cv_.wait(lk, [&]{
          return !rx_queue_.empty()
              || static_cast<bool>(async_err_)
              || reader_stop_.load(std::memory_order_relaxed);
        });

        if(!rx_queue_.empty())
        {
          pending_     = std::move(rx_queue_.front());
          rx_queue_.pop_front();
          pending_off_ = 0;
        }
        else if(async_err_)
        {
          err = async_err_;
          async_err_.clear();
          return 0;
        }
        else
        {
          err = boost::asio::error::operation_aborted;
          return 0;
        }
      }

      const size_t avail   = pending_.size() - pending_off_;
      const size_t to_copy = std::min(avail, buf.size());
      std::memcpy(buf.data(), pending_.data() + pending_off_, to_copy);
      pending_off_ += to_copy;
      err = boost::system::error_code();
      return to_copy;
    }

    virtual size_t endpoint_write(boost::asio::const_buffer buf, boost::system::error_code& err)
    {
      const boost::array<boost::asio::const_buffer, 1> bufs = {buf};
      return this->endpoint_.send(bufs, 0, err);
    }

  public:
    ~Oem7ReceiverNet() override
    {
      join_reader_if_any();
    }
  };

  class Oem7ReceiverTcp: public Oem7ReceiverNet<boost::asio::ip::tcp>{};
  class Oem7ReceiverUdp: public Oem7ReceiverNet<boost::asio::ip::udp>{};
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverTcp,     novatel_oem7_driver::Oem7ReceiverIf)
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverUdp,     novatel_oem7_driver::Oem7ReceiverIf)
