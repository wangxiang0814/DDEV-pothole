import json
import tempfile
import unittest
from pathlib import Path

from ddevsim.channels import WHEEL_ORDER, build_trucksim2019_contract, write_contract


class ChannelContractTests(unittest.TestCase):
    def test_four_corner_actuator_channels_are_unique_and_ordered(self):
        contract = build_trucksim2019_contract()
        self.assertEqual(tuple(contract["wheel_order"]), WHEEL_ORDER)

        for group_name in ("wheel_torque", "active_suspension_force"):
            group = contract["imports"][group_name]
            self.assertEqual(tuple(group.keys()), WHEEL_ORDER)
            names = [group[wheel]["trucksim_name"] for wheel in WHEEL_ORDER]
            self.assertEqual(len(names), 4)
            self.assertEqual(len(set(names)), 4)

    def test_contract_uses_same_vehicle_wheel_torque_and_active_force_interfaces(self):
        contract = build_trucksim2019_contract()
        torque_names = [
            contract["imports"]["wheel_torque"][wheel]["trucksim_name"]
            for wheel in WHEEL_ORDER
        ]
        force_names = [
            contract["imports"]["active_suspension_force"][wheel]["trucksim_name"]
            for wheel in WHEEL_ORDER
        ]
        self.assertEqual(torque_names, ["IMP_MYUSM_L1", "IMP_MYUSM_R1", "IMP_MYUSM_L2", "IMP_MYUSM_R2"])
        self.assertEqual(force_names, ["IMP_FS_L1", "IMP_FS_R1", "IMP_FS_L2", "IMP_FS_R2"])
        for channel in contract["imports"]["wheel_torque"].values():
            self.assertEqual(channel["mode"], "ADD")
            self.assertEqual(channel["unit"], "N-m")
        for channel in contract["imports"]["active_suspension_force"].values():
            self.assertEqual(channel["mode"], "ADD")
            self.assertEqual(channel["unit"], "N")
        self.assertNotIn("brake_pressure", contract["imports"])
        for group in contract["imports"].values():
            for channel in group.values():
                self.assertEqual(channel["verification_status"], "same_vehicle_pulse_passed")

    def test_serialized_contract_is_stable_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.json"
            write_contract(path)
            loaded = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(loaded["schema_version"], "2.0")
            self.assertTrue(loaded["limitations"])


if __name__ == "__main__":
    unittest.main()
