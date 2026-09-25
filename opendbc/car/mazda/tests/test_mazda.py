import unittest

from opendbc.can import CANPacker
from opendbc.car.mazda import mazdacan


class TestMazdaCan(unittest.TestCase):
  def test_crz_info(self):
    """CRZ_INFO matches stock, checksum included"""
    packer = CANPacker("mazda_2017")
    # (stock frame, accel, active, available, stopping, resume)
    for frame, accel, active, available, stopping, resume in [
      ("01ffe3ffc000025b", 4.094, False, False, False, False),  # main off
      ("01ffe3ffc4800acf", 4.094, False, True, False, False),  # armed
      ("01ffe20266800e27", 0.019, True, True, False, False),  # engaged
      ("01ffe1dc66841a42", -0.285, True, True, True, False),  # stopping
      ("01ffe1ffe68040b9", -0.001, True, True, False, True),  # releasing Auto Hold
    ]:
      with self.subTest(frame=frame):
        dat = bytes.fromhex(frame)
        msg = mazdacan.create_acc_command(packer, 0, dat[6] & 0xf, accel, active, available, stopping, resume)
        self.assertEqual(msg[1], dat)


if __name__ == "__main__":
  unittest.main()
