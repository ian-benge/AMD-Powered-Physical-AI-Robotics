"""Compile every runtime module, including ROS adapters unavailable on Windows."""
import ast
from pathlib import Path
import unittest


class SyntaxTests(unittest.TestCase):
    def test_all_runtime_modules_compile(self):
        root = Path(__file__).resolve().parents[1]
        for path in root.rglob('*.py'):
            with self.subTest(path=path.name):
                ast.parse(path.read_text(), filename=str(path))


if __name__ == '__main__':
    unittest.main()
