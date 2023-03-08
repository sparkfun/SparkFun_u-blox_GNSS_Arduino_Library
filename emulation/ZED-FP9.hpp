#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

#define CAT_FIELDS(MSB, LSB) ((MSB << 8) + LSB)

static bool read_buffer_ok(int fd, uint8_t* buf, size_t size, int tries=3)
{
	auto bytes_remaining = size;

	for (tries = 3; bytes_remaining > 0 && tries--;)
	{
		auto read_res = read(fd, buf + size - bytes_remaining, bytes_remaining);
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
	std::vector<uint8_t> payload;
	Footer footer;

	void compute_checksums(uint8_t& checksumA, uint8_t& checksumB) const
	{
		checksumA = 0;
		checksumB = 0;

		checksumA += header.cls;
		checksumB += checksumA;

		checksumA += header.id;
		checksumB += checksumA;

		checksumA += (playload.size() & 0xFF);
		checksumB += checksumA;

		checksumA += (len >> 8);
		checksumB += checksumA;

		for (uint16_t i = 0; i < playload.size(); i++)
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

	static UbloxPacket read_from_fd(int fd)
	{
		UbloxPacket packet;
		// read(fd, &packet.header, sizeof(packet.header));
		assert(read_buffer_ok(fd, &packet.header, sizeof(packet.header)));
		packet.payload.reserve(packet.header.len);
		assert(read_buffer_ok(fd, packet.payload.data(), packet.header.len));
		assert(!read_buffer_ok(fd, &packet.footer, sizeof(footer)));

		return packet;
	}
};


struct ZED_FP9
{
	void service(int fd)
	{
		process_packet(UbloxPacket::read_from_fd(fd));
	}

private:
	void process_packet(int fd, UbloxHeader& packet)
	{
		switch(packet.key())
		{
			case CAT_FIELDS(UBX_CLASS_CFG, UBX_CFG_PRT):
				// process port config
				break;

		}
	}

	std::queue<UbloxPacket> m_packet_queue;
};