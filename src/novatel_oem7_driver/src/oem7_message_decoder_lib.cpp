#include <oem7_message_decoder_lib.hpp>

#include <decoders/novatel/api/framer.hpp>
#include <decoders/novatel/api/common.hpp>
#include <decoders/common/api/common.hpp>
#include <logger/logger.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace
{
  static const novatel_oem7::version_element_t VERSION_MAJOR  = 10;
  static const novatel_oem7::version_element_t VERSION_MINOR  = 2;
  static const novatel_oem7::version_element_t VERSION_SPECIAL= 0;

  // Frame buffer must hold the largest possible OEM message.
  // MAX_BINARY_MESSAGE_LENGTH covers binary; ASCII responses can be longer,
  // so use the larger of the message-class macros.
  constexpr size_t kFrameBufferSize = MAX_ASCII_MESSAGE_LENGTH;

  // Scratch buffer used to pull bytes from the receiver before pushing into
  // the framer. Sized to amortize syscall overhead without bloating memory.
  constexpr size_t kReadBufferSize = 4096;
}

namespace novatel_oem7
{
  /**
   * Wraps a single framed OEM message: the raw frame bytes plus the
   * MetaDataStruct populated by the EDIE Framer.
   */
  class Oem7RawMessage : public Oem7RawMessageIf
  {
    std::vector<uint8_t>           frame_;
    novatel::edie::oem::MetaDataStruct  meta_;

  public:
    Oem7RawMessage(const uint8_t* data, size_t length, const novatel::edie::oem::MetaDataStruct& meta)
    : frame_(data, data + length), meta_(meta)
    {
    }

    Oem7MessageType getMessageType() const override
    {
      return meta_.bResponse ? OEM7MSGTYPE_RSP : OEM7MSGTYPE_LOG;
    }

    Oem7MessageFormat getMessageFormat() const override
    {
      switch (meta_.eFormat)
      {
        case novatel::edie::HEADERFORMAT::BINARY:        return OEM7MSGFMT_BINARY;
        case novatel::edie::HEADERFORMAT::SHORT_BINARY:  return OEM7MSGFMT_SHORTBINARY;
        case novatel::edie::HEADERFORMAT::ASCII:         return OEM7MSGFMT_ASCII;
        case novatel::edie::HEADERFORMAT::ABB_ASCII:     return OEM7MSGFMT_ABASCII;
        default:                                         return OEM7MSGFMT_UNKNOWN;
      }
    }

    int getMessageId() const override
    {
      return meta_.usMessageID;
    }

    const uint8_t* getMessageData(size_t offset) const override
    {
      return frame_.data() + offset;
    }

    size_t getMessageDataLength() const override
    {
      return frame_.size();
    }
  };


  /**
   * Decoder library implementation: wraps the EDIE v3 push-based framer.
   * The user pulls bytes via Oem7MessageDecoderLibUserIf::read; we push them
   * into the framer and harvest framed messages with GetFrame.
   */
  class Oem7MessageDecoderLib : public Oem7MessageDecoderLibIf
  {
    Oem7MessageDecoderLibUserIf*           user_;
    novatel::edie::oem::Framer             framer_;
    std::array<uint8_t, kReadBufferSize>   read_buffer_{};
    std::array<uint8_t, kFrameBufferSize>  frame_buffer_{};

  public:
    explicit Oem7MessageDecoderLib(Oem7MessageDecoderLibUserIf* user) : user_(user)
    {
      framer_.SetFrameJson(false);
      framer_.SetPayloadOnly(false);
    }

    bool readMessage(std::shared_ptr<Oem7RawMessageIf>& msg) override
    {
      while (true)
      {
        novatel::edie::oem::MetaDataStruct meta;
        const novatel::edie::STATUS status =
          framer_.GetFrame(frame_buffer_.data(), frame_buffer_.size(), meta);

        if (status == novatel::edie::STATUS::SUCCESS)
        {
          msg = std::make_shared<Oem7RawMessage>(frame_buffer_.data(), meta.uiLength, meta);
          return true;
        }

        // For BUFFER_EMPTY / INCOMPLETE / INCOMPLETE_MORE_DATA we need more
        // bytes. UNKNOWN means the framer skipped a junk byte and is ready
        // to keep going; we feed more anyway to make forward progress.
        size_t bytes_read = 0;
        const bool ok = user_->read(
          boost::asio::buffer(read_buffer_.data(), read_buffer_.size()),
          bytes_read);

        if (!ok)
        {
          return false;  // end of stream
        }

        if (bytes_read > 0)
        {
          const size_t safe_n = std::min(bytes_read, read_buffer_.size());
          framer_.Write(read_buffer_.data(), static_cast<uint32_t>(safe_n));
        }
      }
    }
  };


  std::shared_ptr<Oem7MessageDecoderLibIf>
  GetOem7MessageDecoder(Oem7MessageDecoderLibUserIf* user)
  {
    // EDIE's Framer constructor registers a logger via Logger::RegisterLogger,
    // which dereferences a static root logger initialized by Logger::InitLogger.
    // Call once before constructing any Framer.
    static std::once_flag init_logger_once;
    std::call_once(init_logger_once, []() { Logger::InitLogger(); });

    return std::make_shared<Oem7MessageDecoderLib>(user);
  }

  void
  GetOem7MessageDecoderLibVersion(version_element_t& major, version_element_t& minor, version_element_t& spec)
  {
    major = VERSION_MAJOR;
    minor = VERSION_MINOR;
    spec  = VERSION_SPECIAL;
  }
}
