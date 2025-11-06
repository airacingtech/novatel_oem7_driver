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

#include <boost/asio.hpp>
#include <pcap/pcap.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace novatel_oem7_driver
{
  /**
   * 'Virtual' Oem7 interface, where input is read from a PCAP file.
   * This replays TCP/UDP packet data from a packet capture file.
   */
  class Oem7ReceiverPcap: public Oem7ReceiverIf
  {
    rclcpp::Node* node_;

    pcap_t* pcap_handle_; ///< PCAP file handle
    size_t num_bytes_read_; ///< Total number of bytes read from PCAP
    size_t num_packets_processed_; ///< Total number of packets processed

    std::string pcap_file_name_; ///< PCAP file path
    std::string target_ip_; ///< Target IP to filter (optional)
    int target_port_; ///< Target port to filter (optional)
    bool verbose_; ///< Enable verbose logging
    double playback_rate_; ///< Playback speed multiplier (1.0 = realtime, 0.5 = half speed, 2.0 = double speed)

    // Buffer for reassembling packet stream
    std::vector<uint8_t> stream_buffer_;
    size_t stream_buffer_pos_;

    // Timing control
    bool first_packet_;
    struct timeval last_packet_time_;

  public:
    Oem7ReceiverPcap():
      pcap_handle_(nullptr),
      num_bytes_read_(0),
      num_packets_processed_(0),
      target_port_(0),
      verbose_(false),
      playback_rate_(1.0),
      stream_buffer_pos_(0),
      first_packet_(true)
    {
      last_packet_time_.tv_sec = 0;
      last_packet_time_.tv_usec = 0;
    }

    ~Oem7ReceiverPcap()
    {
      if(pcap_handle_)
      {
        pcap_close(pcap_handle_);
      }
    }

    /**
     * Opens and prepares the PCAP file for reading.
     */
    virtual bool initialize(rclcpp::Node& nh)
    {
      node_ = &nh;

      // Declare and get parameters
      node_->declare_parameter("oem7_pcap_file", "");
      node_->declare_parameter("oem7_ip_addr", "");
      node_->declare_parameter("oem7_port", 0);
      node_->declare_parameter("oem7_pcap_verbose", false);
      node_->declare_parameter("oem7_pcap_rate", 1.0);

      pcap_file_name_ = node_->get_parameter("oem7_pcap_file").as_string();
      target_ip_ = node_->get_parameter("oem7_ip_addr").as_string();
      target_port_ = node_->get_parameter("oem7_port").as_int();
      verbose_ = node_->get_parameter("oem7_pcap_verbose").as_bool();
      playback_rate_ = node_->get_parameter("oem7_pcap_rate").as_double();

      if(verbose_)
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), 
                           "Oem7Pcap['" << pcap_file_name_ << "'] "
                           << "IP: '" << target_ip_ << "' Port: " << target_port_);
      }

      // Open PCAP file
      char errbuf[PCAP_ERRBUF_SIZE];
      pcap_handle_ = pcap_open_offline(pcap_file_name_.c_str(), errbuf);
      
      if(!pcap_handle_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), 
                            "Could not open PCAP file '" << pcap_file_name_ 
                            << "': " << errbuf);
        return false;
      }

      if(verbose_)
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully opened PCAP file");
      }
      return true;
    }

    /**
     * Reads the next packet from the PCAP file and extracts TCP/UDP payload.
     */
    virtual bool read(boost::asio::mutable_buffer buf, size_t& rlen)
    {
      if(!pcap_handle_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "PCAP handle is not initialized");
        return false;
      }

      // Try to fill the buffer from PCAP packets
      size_t bytes_to_read = boost::asio::buffer_size(buf);
      uint8_t* buffer_ptr = boost::asio::buffer_cast<uint8_t*>(buf);
      size_t bytes_written = 0;

      // First, drain any buffered data
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

      // Read next packet from PCAP - just ONE packet per read() call
      while(rclcpp::ok())
      {
        struct pcap_pkthdr* header;
        const u_char* packet_data;
        
        int result = pcap_next_ex(pcap_handle_, &header, &packet_data);
        
        if(result == -2) // End of file
        {
          if(verbose_)
          {
            RCLCPP_INFO_STREAM(node_->get_logger(), 
                               "End of PCAP file. Processed " << num_packets_processed_ 
                               << " packets, " << num_bytes_read_ << " bytes total");
          }
          return false;
        }
        else if(result == -1) // Error
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), 
                              "Error reading PCAP: " << pcap_geterr(pcap_handle_));
          return false;
        }
        else if(result == 0) // Timeout (shouldn't happen with offline files)
        {
          continue;
        }

        // Parse the packet and extract payload
        size_t payload_size = 0;
        const uint8_t* payload = extract_payload(packet_data, header->caplen, payload_size);
        
        if(payload && payload_size > 0)
        {
          // Handle timing - sleep BEFORE processing this packet to match PCAP timestamps
          if(!first_packet_)
          {
            // Calculate time delta from last packet
            long delta_sec = header->ts.tv_sec - last_packet_time_.tv_sec;
            long delta_usec = header->ts.tv_usec - last_packet_time_.tv_usec;
            long delta_total_usec = delta_sec * 1000000 + delta_usec;
            
            // Apply playback rate and sleep to match the actual packet timing
            if(delta_total_usec > 0)
            {
              long sleep_usec = static_cast<long>(delta_total_usec / playback_rate_);
              usleep(sleep_usec);
            }
          }
          
          // Update last packet time
          last_packet_time_ = header->ts;
          first_packet_ = false;
          
          num_packets_processed_++;
          
          // Return this packet's payload - just like a real socket would
          if(payload_size <= bytes_to_read)
          {
            std::memcpy(buffer_ptr, payload, payload_size);
            rlen = payload_size;
            num_bytes_read_ += payload_size;
            return true;
          }
          else
          {
            // Payload is larger than buffer - copy what fits, buffer the rest
            std::memcpy(buffer_ptr, payload, bytes_to_read);
            stream_buffer_.assign(payload + bytes_to_read, payload + payload_size);
            stream_buffer_pos_ = 0;
            rlen = bytes_to_read;
            num_bytes_read_ += bytes_to_read;
            return true;
          }
        }
        // If packet didn't match our filter, continue to next packet
      }

      return false;
    }

    /**
     * Takes no action (PCAP replay is read-only).
     *
     * @return false always.
     */
    virtual bool write(boost::asio::const_buffer buf)
    {
      return false;
    }

  private:
    /**
     * Extracts TCP/UDP payload from a packet.
     * Returns pointer to payload and sets payload_size.
     */
    const uint8_t* extract_payload(const uint8_t* packet, size_t packet_len, size_t& payload_size)
    {
      payload_size = 0;

      // Check minimum ethernet header size
      if(packet_len < sizeof(struct ether_header))
      {
        return nullptr;
      }

      // Parse Ethernet header
      struct ether_header* eth_header = (struct ether_header*)packet;
      uint16_t ether_type = ntohs(eth_header->ether_type);

      size_t offset = sizeof(struct ether_header);

      // Only process IPv4 packets
      if(ether_type != ETHERTYPE_IP)
      {
        return nullptr;
      }

      // Check for IP header
      if(packet_len < offset + sizeof(struct iphdr))
      {
        return nullptr;
      }

      // Parse IP header
      struct iphdr* ip_header = (struct iphdr*)(packet + offset);
      
      // Only process TCP and UDP packets
      if(ip_header->protocol != IPPROTO_TCP && ip_header->protocol != IPPROTO_UDP)
      {
        return nullptr;
      }

      // Filter by IP address if specified
      if(!target_ip_.empty())
      {
        struct in_addr src_addr, dst_addr;
        src_addr.s_addr = ip_header->saddr;
        dst_addr.s_addr = ip_header->daddr;
        
        std::string src_ip = inet_ntoa(src_addr);
        std::string dst_ip = inet_ntoa(dst_addr);
        
        if(src_ip != target_ip_ && dst_ip != target_ip_)
        {
          return nullptr;
        }
      }

      size_t ip_header_len = ip_header->ihl * 4;
      offset += ip_header_len;

      // Handle TCP packets
      if(ip_header->protocol == IPPROTO_TCP)
      {
        // Check for TCP header
        if(packet_len < offset + sizeof(struct tcphdr))
        {
          return nullptr;
        }

        // Parse TCP header
        struct tcphdr* tcp_header = (struct tcphdr*)(packet + offset);
        
        // Filter by port if specified
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
      else // UDP
      {
        // Check for UDP header
        if(packet_len < offset + sizeof(struct udphdr))
        {
          return nullptr;
        }

        // Parse UDP header
        struct udphdr* udp_header = (struct udphdr*)(packet + offset);
        
        // Filter by port if specified
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

      // Calculate payload size
      if(packet_len <= offset)
      {
        return nullptr; // No payload
      }

      payload_size = packet_len - offset;
      return packet + offset;
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverPcap, novatel_oem7_driver::Oem7ReceiverIf)
