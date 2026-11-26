"""Prevent accidental overwriting of experiment evidence before any simulator launch."""

from pathlib import Path
import subprocess
import sys


def test_existing_result_is_not_overwritten(tmp_path):
    path = tmp_path / "retained"
    path.with_suffix(".json").write_text('{"retained": true}\n')
    script = Path(__file__).parents[1] / "scripts/run_continuous.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--webots",
            "/does/not/exist",
            "--mode",
            "evaluate",
            "--output",
            str(path),
            "--seed",
            "4400000",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2 and "Existing experiment output" in result.stderr
    assert path.with_suffix(".json").read_text() == '{"retained": true}\n'
    assert not path.with_suffix(".launch.json").exists()
