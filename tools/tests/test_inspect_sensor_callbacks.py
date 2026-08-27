import unittest

from tools.inspect_sensor_callbacks import EXPECTED_CALLBACKS, callback_offsets


class CallbackOffsetTests(unittest.TestCase):
    def test_reconstructs_interleaved_literal_stores(self) -> None:
        lines: list[str] = []
        relocations: list[str] = []
        registers = ("r2", "r3")
        for index, (symbol, destination) in enumerate(EXPECTED_CALLBACKS.items()):
            register = registers[index % len(registers)]
            literal = 0x800 + index * 4
            lines.append(
                f" 700:\t4a01\tldr\t{register}, [pc, #4]\t@ "
                f"({literal:x} <init+0x100>)"
            )
            lines.append(
                f" 704:\tf8c4\tstr.w\t{register}, [r4, #{destination}]\t@ "
                f"0x{destination:x}"
            )
            relocations.append(f"\t\t\t{literal:x}: R_ARM_ABS32\t{symbol}")

        observed = callback_offsets("\n".join(lines + relocations))
        self.assertEqual(observed, EXPECTED_CALLBACKS)

    def test_ignores_unrelated_relocations(self) -> None:
        disassembly = "\n".join(
            (
                " 700:\t4a01\tldr\tr2, [pc, #4]\t@ (800 <init+0x100>)",
                " 704:\tf8c4\tstr.w\tr2, [r4, #2500]\t@ 0x9c4",
                "\t\t\t800: R_ARM_ABS32\tunrelated_symbol",
            )
        )
        self.assertEqual(callback_offsets(disassembly), {})


if __name__ == "__main__":
    unittest.main()
