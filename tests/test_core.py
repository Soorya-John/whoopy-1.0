import pathlib
import subprocess
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class CoreTests(unittest.TestCase):
    def test_protocol_and_control_core(self):
        exe = ROOT / "tests/.core_tests.exe"
        try:
            cmd = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "shared/control"), "-I", str(ROOT / "shared/protocol"), str(ROOT / "tests/test_core.c"), str(ROOT / "shared/control/fc_core.c"), "-lm", "-o", str(exe)]
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
            self.assertIn("core tests passed", result.stdout)
        finally:
            exe.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
