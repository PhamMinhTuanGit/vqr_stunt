import struct
import unittest

import mujoco
import numpy as np

from mujoco_simulation import (
    MuJoCoSimulation,
    STATE_FLOAT_COUNT,
    STATE_PACKET_SIZE,
)


class _PacketSink:
    def __init__(self):
        self.packet = None

    def sendto(self, packet, _address):
        self.packet = packet


class MuJoCoStateProtocolTest(unittest.TestCase):
    def test_headless_model_and_v2_packet(self):
        simulation = MuJoCoSimulation(use_viewer=False, enable_udp=False)
        sink = _PacketSink()
        simulation.send_sock = sink
        try:
            for _ in range(10):
                simulation._apply_joint_torque()
                mujoco.mj_step(simulation.model, simulation.data)
            simulation.timestamp = 0.01
            simulation._send_robot_state(10)

            self.assertIsNotNone(sink.packet)
            self.assertEqual(len(sink.packet), STATE_PACKET_SIZE)
            values = struct.unpack(f"<d{STATE_FLOAT_COUNT}f", sink.packet)
            payload = np.asarray(values[1:], dtype=np.float64)
            self.assertTrue(np.isfinite(payload).all())
            self.assertAlmostEqual(np.linalg.norm(payload[15:19]), 1.0, places=5)
            self.assertTrue((payload[19:23] >= 0.0).all())
        finally:
            simulation.send_sock = None


if __name__ == "__main__":
    unittest.main()
