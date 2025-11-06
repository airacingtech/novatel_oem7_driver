////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2025 AI Racing Tech
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

#include <boost/asio.hpp>
#include <pcap/pcap.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <mutex>

namespace novatel_oem7_driver
{
  class Oem7ReceiverPcap: public Oem7ReceiverIf
  {
    // Playback control constants
    static constexpr double SPEED_INCREMENT = 0.10;
    static constexpr double MIN_PLAYBACK_SPEED = 0.10;
    static constexpr double MAX_PLAYBACK_SPEED = 10.0;
    static constexpr double SEEK_INTERVAL_SECONDS = 5.0;
    static constexpr int KEYBOARD_POLL_INTERVAL_MS = 50;

    rclcpp::Node* node_;

    pcap_t* pcap_handle_;
    size_t num_bytes_read_;
    size_t num_packets_processed_;

    std::string pcap_file_name_;
    std::string target_ip_;
    int target_port_;
    double playback_rate_;

    std::vector<uint8_t> stream_buffer_;
    size_t stream_buffer_pos_;

    bool first_packet_;
    struct timeval last_packet_time_;
    std::mutex last_packet_time_mutex_;
    std::atomic<double> first_pcap_timestamp_;
    
    std::atomic<bool> paused_;
    std::atomic<double> playback_speed_;
    std::atomic<double> seek_target_;
    std::atomic<bool> seek_requested_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
    int tty_fd_;
    struct termios orig_termios_;

  public:
    Oem7ReceiverPcap():
      pcap_handle_(nullptr),
      num_bytes_read_(0),
      num_packets_processed_(0),
      target_port_(0),
      playback_rate_(1.0),
      stream_buffer_pos_(0),
      first_packet_(true),
      first_pcap_timestamp_(0.0),
      paused_(false),
      playback_speed_(1.0),
      seek_target_(0.0),
      seek_requested_(false),
      running_(false),
      tty_fd_(-1)
    {
      last_packet_time_.tv_sec = 0;
      last_packet_time_.tv_usec = 0;
    }

    ~Oem7ReceiverPcap()
    {
      running_ = false;
      if(keyboard_thread_.joinable())
      {
        keyboard_thread_.join();
      }
      restoreTerminal();
      if(pcap_handle_)
      {
        pcap_close(pcap_handle_);
      }
    }

    virtual bool initialize(rclcpp::Node& nh)
    {
      node_ = &nh;

      node_->declare_parameter("oem7_pcap_file", "");
      node_->declare_parameter("oem7_ip_addr", "");
      node_->declare_parameter("oem7_port", 0);
      node_->declare_parameter("oem7_pcap_rate", 1.0);

      pcap_file_name_ = node_->get_parameter("oem7_pcap_file").as_string();
      target_ip_ = node_->get_parameter("oem7_ip_addr").as_string();
      target_port_ = node_->get_parameter("oem7_port").as_int();
      playback_rate_ = node_->get_parameter("oem7_pcap_rate").as_double();

      RCLCPP_INFO_STREAM(node_->get_logger(), 
                         "Oem7Pcap['" << pcap_file_name_ << "'] "
                         << "IP: '" << target_ip_ << "' Port: " << target_port_);

      char errbuf[PCAP_ERRBUF_SIZE];
      pcap_handle_ = pcap_open_offline(pcap_file_name_.c_str(), errbuf);
      
      if(!pcap_handle_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), 
                            "Could not open PCAP file '" << pcap_file_name_ 
                            << "': " << errbuf);
        return false;
      }

      RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully opened PCAP file");
      
      setupTerminal();
      running_ = true;
      playback_speed_.store(playback_rate_);
      keyboard_thread_ = std::thread(&Oem7ReceiverPcap::keyboardInputThread, this);
      
      return true;
    }

    void setupTerminal()
    {
      tty_fd_ = ::open("/dev/tty", O_RDWR | O_NONBLOCK);
      if (tty_fd_ < 0)
      {
        return;
      }
      
      tcgetattr(tty_fd_, &orig_termios_);
      
      struct termios raw = orig_termios_;
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 1;
      tcsetattr(tty_fd_, TCSANOW, &raw);
    }

    void restoreTerminal()
    {
      if (tty_fd_ >= 0)
      {
        tcsetattr(tty_fd_, TCSANOW, &orig_termios_);
        ::close(tty_fd_);
        tty_fd_ = -1;
      }
    }

    void keyboardInputThread()
    {
      if (tty_fd_ < 0)
      {
        return;
      }
      
      RCLCPP_INFO(node_->get_logger(), 
        "\n"
        "=======================================================\n"
        "  PCAP Playback Controls:\n"
        "  SPACE     - Pause/Resume\n"
        "  UP        - Increase speed (0.10x increments)\n"
        "  DOWN      - Decrease speed (0.10x increments)\n"
        "  LEFT (<)  - Seek backward 5 seconds\n"
        "  RIGHT (>) - Seek forward 5 seconds\n"
        "  q         - Quit\n"
        "=======================================================\n");

      while (running_)
      {
        char c;
        if (::read(tty_fd_, &c, 1) == 1)
        {
          if (c == ' ')
          {
            bool was_paused = paused_.load();
            paused_.store(!was_paused);
            RCLCPP_INFO(node_->get_logger(), 
              was_paused ? "[RESUME] Playback resumed" : "[PAUSE] Playback paused");
          }
          else if (c == 27)
          {
            char seq[2];
            if (::read(tty_fd_, &seq[0], 1) == 1 && 
                ::read(tty_fd_, &seq[1], 1) == 1)
            {
              if (seq[0] == '[')
              {
                switch (seq[1])
                {
                  case 'A': {
                    double current_speed = playback_speed_.load();
                    double new_speed = std::min(current_speed + SPEED_INCREMENT, MAX_PLAYBACK_SPEED);
                    playback_speed_.store(new_speed);
                    RCLCPP_INFO(node_->get_logger(), 
                      "[SPEED] Playback speed: %.2fx", new_speed);
                    break;
                  }
                  case 'B': {
                    double current_speed = playback_speed_.load();
                    double new_speed = std::max(current_speed - SPEED_INCREMENT, MIN_PLAYBACK_SPEED);
                    playback_speed_.store(new_speed);
                    RCLCPP_INFO(node_->get_logger(), 
                      "[SPEED] Playback speed: %.2fx", new_speed);
                    break;
                  }
                  case 'D': {
                    double current_relative;
                    {
                      std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
                      current_relative = (last_packet_time_.tv_sec + last_packet_time_.tv_usec / 1000000.0) - first_pcap_timestamp_.load();
                    }
                    double target_relative = std::max(0.0, current_relative - SEEK_INTERVAL_SECONDS);
                    seek_target_.store(target_relative);
                    seek_requested_.store(true);
                    RCLCPP_INFO(node_->get_logger(), 
                      "[SEEK] Seeking backward %.0fs to %.1fs", SEEK_INTERVAL_SECONDS, target_relative);
                    break;
                  }
                  case 'C': {
                    double current_relative;
                    {
                      std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
                      current_relative = (last_packet_time_.tv_sec + last_packet_time_.tv_usec / 1000000.0) - first_pcap_timestamp_.load();
                    }
                    double target_relative = current_relative + SEEK_INTERVAL_SECONDS;
                    seek_target_.store(target_relative);
                    seek_requested_.store(true);
                    RCLCPP_INFO(node_->get_logger(), 
                      "[SEEK] Seeking forward %.0fs to %.1fs", SEEK_INTERVAL_SECONDS, target_relative);
                    break;
                  }
                }
              }
            }
          }
          else if (c == 'q' || c == 'Q')
          {
            RCLCPP_INFO(node_->get_logger(), "[QUIT] Stopping PCAP playback...");
            running_ = false;
            break;
          }
          else if (c == '<' || c == ',')
          {
            double current_relative;
            {
              std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
              current_relative = (last_packet_time_.tv_sec + last_packet_time_.tv_usec / 1000000.0) - first_pcap_timestamp_.load();
            }
            double target_relative = std::max(0.0, current_relative - SEEK_INTERVAL_SECONDS);
            seek_target_.store(target_relative);
            seek_requested_.store(true);
            RCLCPP_INFO(node_->get_logger(), 
              "[SEEK] Seeking backward %.0fs to %.1fs", SEEK_INTERVAL_SECONDS, target_relative);
          }
          else if (c == '>' || c == '.')
          {
            double current_relative;
            {
              std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
              current_relative = (last_packet_time_.tv_sec + last_packet_time_.tv_usec / 1000000.0) - first_pcap_timestamp_.load();
            }
            double target_relative = current_relative + SEEK_INTERVAL_SECONDS;
            seek_target_.store(target_relative);
            seek_requested_.store(true);
            RCLCPP_INFO(node_->get_logger(), 
              "[SEEK] Seeking forward %.0fs to %.1fs", SEEK_INTERVAL_SECONDS, target_relative);
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(KEYBOARD_POLL_INTERVAL_MS));
      }
    }

    virtual bool read(boost::asio::mutable_buffer buf, size_t& rlen)
    {
      if(!pcap_handle_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "PCAP handle is not initialized");
        return false;
      }

      size_t bytes_to_read = boost::asio::buffer_size(buf);
      uint8_t* buffer_ptr = boost::asio::buffer_cast<uint8_t*>(buf);
      size_t bytes_written = 0;

      if(stream_buffer_pos_ < stream_buffer_.size())
      {
        size_t available = stream_buffer_.size() - stream_buffer_pos_;
        size_t to_copy = std::min(available, bytes_to_read);
        
        std::memcpy(buffer_ptr, 
                    stream_buffer_.data() + stream_buffer_pos_, 
                    to_copy);
        
        stream_buffer_pos_ += to_copy;
        
        if(stream_buffer_pos_ >= stream_buffer_.size())
        {
          stream_buffer_.clear();
          stream_buffer_pos_ = 0;
        }
        
        rlen = to_copy;
        num_bytes_read_ += to_copy;
        return true;
      }

      while(rclcpp::ok())
      {
        while (paused_.load() && running_)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!running_)
        {
          return false;
        }
        
        struct pcap_pkthdr* header;
        const u_char* packet_data;
        
        int result = pcap_next_ex(pcap_handle_, &header, &packet_data);
        
        if(result == -2)
        {
          RCLCPP_INFO_STREAM(node_->get_logger(), 
                             "End of PCAP file. Processed " << num_packets_processed_ 
                             << " packets, " << num_bytes_read_ << " bytes total");
          return false;
        }
        else if(result == -1)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), 
                              "Error reading PCAP: " << pcap_geterr(pcap_handle_));
          return false;
        }
        else if(result == 0)
        {
          continue;
        }

        double current_pcap_timestamp = header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
        
        if(first_packet_)
        {
          first_pcap_timestamp_.store(current_pcap_timestamp);
          first_packet_ = false;
        }
        
        if(seek_requested_.load())
        {
          double target_relative = seek_target_.load();
          double target_absolute = first_pcap_timestamp_.load() + target_relative;
          seek_requested_.store(false);
          
          if(target_absolute < current_pcap_timestamp)
          {
            pcap_close(pcap_handle_);
            char errbuf[PCAP_ERRBUF_SIZE];
            pcap_handle_ = pcap_open_offline(pcap_file_name_.c_str(), errbuf);
            
            if(!pcap_handle_)
            {
              RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to reopen PCAP file: " << errbuf);
              return false;
            }
            
            first_packet_ = true;
            stream_buffer_.clear();
            stream_buffer_pos_ = 0;
            
            while(running_ && (result = pcap_next_ex(pcap_handle_, &header, &packet_data)) >= 0)
            {
              if(result == 0) continue;
              
              double ts = header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
              
              if(first_packet_)
              {
                first_pcap_timestamp_.store(ts);
                first_packet_ = false;
                target_absolute = first_pcap_timestamp_.load() + target_relative;
              }
              
              if(ts >= target_absolute)
              {
                current_pcap_timestamp = ts;
                {
                  std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
                  last_packet_time_ = header->ts;
                }
                RCLCPP_INFO(node_->get_logger(), "[SEEK] Jumped to %.1fs", target_relative);
                break;
              }
            }
            continue;
          }
          
          if(target_absolute > current_pcap_timestamp)
          {
            while(running_ && (result = pcap_next_ex(pcap_handle_, &header, &packet_data)) >= 0)
            {
              if(result == 0) continue;
              
              double ts = header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
              if(ts >= target_absolute)
              {
                current_pcap_timestamp = ts;
                {
                  std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
                  last_packet_time_ = header->ts;
                }
                RCLCPP_INFO(node_->get_logger(), "[SEEK] Jumped to %.1fs", target_relative);
                break;
              }
            }
            continue;
          }
        }

        size_t payload_size = 0;
        const uint8_t* payload = extract_payload(packet_data, header->caplen, payload_size);
        
        if(payload && payload_size > 0)
        {
          long delta_total_usec = 0;
          {
            std::lock_guard<std::mutex> lock(last_packet_time_mutex_);
            if(last_packet_time_.tv_sec != 0)
            {
              long delta_sec = header->ts.tv_sec - last_packet_time_.tv_sec;
              long delta_usec = header->ts.tv_usec - last_packet_time_.tv_usec;
              delta_total_usec = delta_sec * 1000000 + delta_usec;
            }
            last_packet_time_ = header->ts;
          }
          
          if(delta_total_usec > 0)
          {
            double current_speed = playback_speed_.load();
            long sleep_usec = static_cast<long>(delta_total_usec / current_speed);
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_usec));
          }
          
          num_packets_processed_++;
          
          if(payload_size <= bytes_to_read)
          {
            std::memcpy(buffer_ptr, payload, payload_size);
            rlen = payload_size;
            num_bytes_read_ += payload_size;
            return true;
          }
          else
          {
            std::memcpy(buffer_ptr, payload, bytes_to_read);
            stream_buffer_.assign(payload + bytes_to_read, payload + payload_size);
            stream_buffer_pos_ = 0;
            rlen = bytes_to_read;
            num_bytes_read_ += bytes_to_read;
            return true;
          }
        }
      }

      return false;
    }

    virtual bool write(boost::asio::const_buffer buf)
    {
      return false;
    }

  private:
    const uint8_t* extract_payload(const uint8_t* packet, size_t packet_len, size_t& payload_size)
    {
      payload_size = 0;

      if(packet_len < sizeof(struct ether_header))
      {
        return nullptr;
      }

      const struct ether_header* eth_header = reinterpret_cast<const struct ether_header*>(packet);
      uint16_t ether_type = ntohs(eth_header->ether_type);
      size_t offset = sizeof(struct ether_header);

      if(ether_type != ETHERTYPE_IP)
      {
        return nullptr;
      }

      if(packet_len < offset + sizeof(struct iphdr))
      {
        return nullptr;
      }

      const struct iphdr* ip_header = reinterpret_cast<const struct iphdr*>(packet + offset);
      uint16_t ip_total_length = ntohs(ip_header->tot_len);
      
      if(ip_header->protocol != IPPROTO_TCP && ip_header->protocol != IPPROTO_UDP)
      {
        return nullptr;
      }

      if(!target_ip_.empty())
      {
        struct in_addr src_addr, dst_addr;
        src_addr.s_addr = ip_header->saddr;
        dst_addr.s_addr = ip_header->daddr;
        
        char src_ip_buf[INET_ADDRSTRLEN];
        char dst_ip_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &src_addr, src_ip_buf, INET_ADDRSTRLEN);
        inet_ntop(AF_INET, &dst_addr, dst_ip_buf, INET_ADDRSTRLEN);
        std::string src_ip(src_ip_buf);
        std::string dst_ip(dst_ip_buf);
        
        if(src_ip != target_ip_ && dst_ip != target_ip_)
        {
          return nullptr;
        }
      }

      size_t ip_header_len = ip_header->ihl * 4;
      offset += ip_header_len;

      if(ip_header->protocol == IPPROTO_TCP)
      {
        if(packet_len < offset + sizeof(struct tcphdr))
        {
          return nullptr;
        }

        const struct tcphdr* tcp_header = reinterpret_cast<const struct tcphdr*>(packet + offset);
        
        if(target_port_ > 0)
        {
          uint16_t src_port = ntohs(tcp_header->source);
          uint16_t dst_port = ntohs(tcp_header->dest);
          
          if(src_port != target_port_ && dst_port != target_port_)
          {
            return nullptr;
          }
        }

        size_t tcp_header_len = tcp_header->doff * 4;
        offset += tcp_header_len;
      }
      else
      {
        if(packet_len < offset + sizeof(struct udphdr))
        {
          return nullptr;
        }

        const struct udphdr* udp_header = reinterpret_cast<const struct udphdr*>(packet + offset);
        
        if(target_port_ > 0)
        {
          uint16_t src_port = ntohs(udp_header->source);
          uint16_t dst_port = ntohs(udp_header->dest);
          
          if(src_port != target_port_ && dst_port != target_port_)
          {
            return nullptr;
          }
        }

        offset += sizeof(struct udphdr);
      }

      // Calculate payload size from IP header length fields
      payload_size = ip_total_length - (ip_header->ihl * 4);
      
      if(ip_header->protocol == IPPROTO_TCP)
      {
        // Get TCP header from earlier offset calculation (not ip_payload_offset)
        size_t tcp_offset = sizeof(struct ether_header) + (ip_header->ihl * 4);
        const struct tcphdr* tcp_header = reinterpret_cast<const struct tcphdr*>(packet + tcp_offset);
        payload_size -= (tcp_header->doff * 4);
      }
      else
      {
        payload_size -= sizeof(struct udphdr);
      }

      // Guard against truncated PCAP frames or malformed headers
      if(payload_size == 0 || offset + payload_size > packet_len)
      {
        return nullptr;
      }

      return packet + offset;
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverPcap, novatel_oem7_driver::Oem7ReceiverIf)
