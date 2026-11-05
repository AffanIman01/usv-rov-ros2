"""Input expiry and command limits for terminal simulation control."""

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "keyboard_control.py"
SPEC = importlib.util.spec_from_file_location("usv_keyboard", SCRIPT)
KEYBOARD = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = KEYBOARD
SPEC.loader.exec_module(KEYBOARD)


class KeyboardControlTest(unittest.TestCase):
    def test_escape_sequences_never_become_movement(self):
        decoder = KEYBOARD.TerminalKeyDecoder()
        state = KEYBOARD.KeyboardCommand()
        # Both common arrow-key encodings, modifier parameters and function keys.
        keys = decoder.feed("\x1b[A\x1b[B\x1b[C\x1b[D\x1bOD\x1b[1;5A\x1b[15~")
        self.assertEqual(keys, [])
        for key in keys:
            state.apply_key(key, 1.0)
        self.assertEqual(state.command(1.1), (0.0, 0.0))

    def test_split_escape_sequences_and_normal_uppercase(self):
        decoder = KEYBOARD.TerminalKeyDecoder()
        self.assertEqual(decoder.feed("\x1b"), [])
        self.assertEqual(decoder.feed("[1;"), [])
        self.assertEqual(decoder.feed("5"), [])
        self.assertEqual(decoder.feed("AW"), ["W"])
        self.assertEqual(decoder.feed("\x1bO"), [])
        self.assertEqual(decoder.feed("Dw"), ["w"])
        self.assertEqual(decoder.feed("\x1bw"), [])
        state = KEYBOARD.KeyboardCommand()
        for key in decoder.feed("W"):
            state.apply_key(key, 1.0)
        self.assertEqual(state.command(1.1), (0.6, 0.0))

    def test_motion_expiry_and_repeat(self):
        state = KEYBOARD.KeyboardCommand()
        state.apply_key("W", 10.0)
        self.assertEqual(state.command(10.2), (0.6, 0.0))
        self.assertEqual(state.command(10.4), (0.0, 0.0))
        state.apply_key("w", 10.5)
        self.assertEqual(state.command(10.6), (0.6, 0.0))

    def test_reversal_turn_signs_and_stop(self):
        state = KEYBOARD.KeyboardCommand()
        for key, expected in [("s", (-0.6, 0.0)), ("a", (0.0, 0.4)),
                              ("d", (0.0, -0.4))]:
            state.apply_key(key, 1.0)
            self.assertEqual(state.command(1.1), expected)
        state.apply_key(" ", 1.2)
        self.assertEqual(state.command(1.21), (0.0, 0.0))

    def test_speed_limits_and_no_keepalive_from_speed_key(self):
        state = KEYBOARD.KeyboardCommand()
        for _ in range(30):
            state.apply_key("+", 1.0)
        state.apply_key("w", 1.0)
        self.assertEqual(state.command(1.1), (1.0, 0.0))
        state.apply_key("a", 1.0)
        self.assertEqual(state.command(1.1), (0.0, 0.6))
        state.apply_key("+", 1.3)
        self.assertEqual(state.command(1.4), (0.0, 0.0))
        for _ in range(30):
            state.apply_key("-", 2.0)
        self.assertEqual((state.speed, state.yaw_speed), (0.1, 0.1))

    def test_quit_unknown_input_and_clock_reversal_stop(self):
        state = KEYBOARD.KeyboardCommand()
        state.apply_key("w", 5.0)
        self.assertEqual(state.command(4.0), (0.0, 0.0))
        state.apply_key("z", 5.1)
        self.assertEqual(state.command(5.2), (0.0, 0.0))
        state.apply_key("w", 6.0)
        self.assertEqual(state.apply_key("q", 6.1), "quit")
        self.assertEqual(state.command(6.2), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
