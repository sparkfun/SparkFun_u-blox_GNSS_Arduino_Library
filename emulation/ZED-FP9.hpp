#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <Arduino.h>

namespace emulation
{

#define CAT_FIELDS(MSB, LSB) ((MSB << 8) + LSB)

template <size_t CAP>
struct Buffer
{
  enum Mode
  {
    LINEAR,
    RING,
  };

  Buffer(Mode mode=LINEAR)
  {
    m_mode = mode;
  }

  void push_back(uint8_t b)
  {
    if (m_mode == LINEAR && m_size < CAP - 1)
    {
      m_buffer[m_left_ptr++] = b;
      m_size++;
    }
    else if (m_mode == RING)
    {
      
    }
  }

  size_t size() const { return m_size; }

  uint8_t& operator[](unsigned i) { return m_buffer[i]; }

  uint8_t* data() { return m_buffer; }

  Mode m_mode;
  uint8_t m_buffer[CAP];
  size_t m_left_ptr = 0, m_right_ptr = 0;
  size_t m_size = 0;
};

static bool read_buffer_ok(Stream& stream, uint8_t* buf, size_t size, int tries=3)
{
	auto bytes_remaining = size;

	for (tries = 3; bytes_remaining > 0 && tries--;)
	{
		auto read_res = stream.readBytes(buf + size - bytes_remaining, bytes_remaining);
		if (read_res > 0)
		{
			bytes_remaining -= read_res;
		} 
		else
		{
			// Read failed, throw ex
			return false;
		}
	}

	return tries >= 0;
}



struct UbloxPacket
{
	struct Header
	{
		uint8_t cls;
		uint8_t id;
		uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
		uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
		uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
	} __attribute__((packed));

	struct Footer
	{
		uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
		uint8_t checksumB;
		sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
		sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
	} __attribute__((packed));

	Header header;
	Buffer<256> payload;
	Footer footer;

	void compute_checksums(uint8_t& checksumA, uint8_t& checksumB) const
	{
		checksumA = 0;
		checksumB = 0;

		checksumA += header.cls;
		checksumB += checksumA;

		checksumA += header.id;
		checksumB += checksumA;

		checksumA += (payload.size() & 0xFF);
		checksumB += checksumA;

		checksumA += (payload.size() >> 8);
		checksumB += checksumA;

		for (uint16_t i = 0; i < payload.size(); i++)
		{
			checksumA += payload[i];
			checksumB += checksumA;
		}
	}

	bool is_valid() const
	{
		uint8_t a, b;
		compute_checksums(a, b);

		return a == footer.checksumA && b == footer.checksumB;
	}

	inline uint16_t key() const
	{
		return CAT_FIELDS(header.cls, header.id);
	}

	static UbloxPacket read_from_stream(Stream& stream)
	{
		UbloxPacket packet;
		// read(fd, &packet.header, sizeof(packet.header));
		read_buffer_ok(stream, (uint8_t*)&packet.header, sizeof(packet.header));
		// packet.payload.reserve(packet.header.len);
		read_buffer_ok(stream, packet.payload.data(), packet.header.len);
		read_buffer_ok(stream, (uint8_t*)&packet.footer, sizeof(footer));

		return packet;
	}
};


struct ZED_FP9
{
	void service(Stream& stream)
	{
		process_packet(stream, UbloxPacket::read_from_stream(stream));
	}

private:
	void process_packet(Stream& stream, const UbloxPacket& packet)
	{
		switch(packet.key())
		{
			case CAT_FIELDS(UBX_CLASS_CFG, UBX_CFG_PRT):
				// process port config
				break;

		}
	}

	//std::queue<UbloxPacket> m_packet_queue;
};

} // namespace emulation