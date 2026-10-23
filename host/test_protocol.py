import struct
import unittest

from protocol import (
    Parser, TYPE_ACK, TYPE_SET_GAINS, TYPE_TELEMETRY, crc16,
    frame, gain_command, telemetry_values,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_fragmented_telemetry_and_signed_q16(self):
        packet = frame(TYPE_TELEMETRY, 19, struct.pack("<8i", 65536, -131072, 3, -4, 5, -6, 7, -8))
        parser = Parser()
        self.assertEqual(parser.feed(packet[:5]), [])
        decoded = parser.feed(packet[5:])
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].sequence, 19)
        values = telemetry_values(decoded[0].payload)
        self.assertEqual(values["theta"], 1.0)
        self.assertEqual(values["dtheta"], -2.0)
        self.assertEqual(values["dalpha_ref"], -8 / 65536)

    def test_crc_error_does_not_emit_packet_and_recovers(self):
        bad = bytearray(frame(TYPE_ACK, 1, b"\x00")); bad[-1] ^= 1
        good = frame(TYPE_ACK, 2, b"\x00")
        packets = Parser().feed(bytes(bad) + good)
        self.assertEqual([(p.frame_type, p.sequence) for p in packets], [(TYPE_ACK, 2)])

    def test_gain_command_encoding_and_limits(self):
        packet = Parser().feed(gain_command(4, [-1.0, 0.0, 1.0, -128.0, 127.999]))[0]
        self.assertEqual(packet.frame_type, TYPE_SET_GAINS)
        self.assertEqual(struct.unpack("<5i", packet.payload), (-1024, 0, 1024, -(1 << 17), round(127.999 * 1024)))
        with self.assertRaises(ValueError):
            gain_command(5, [128.0, 0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
