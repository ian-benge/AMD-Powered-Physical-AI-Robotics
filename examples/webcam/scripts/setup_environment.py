"""Create a local CPU environment; runnable using standard Python alone."""

from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
import sys
import venv

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    # Run every setup command from the project root so generated files and
    # installed checks consistently refer to this checkout.
    print("\nRunning:", " ".join(map(str, command)), flush=True)
    return subprocess.run(list(map(str, command)), cwd=ROOT, check=True, **kwargs)


def main():
    if not (3, 10) <= sys.version_info[:2] < (3, 14):
        raise RuntimeError("Use Python 3.12 (recommended), or 3.10-3.13, for this starter.")
    target = ROOT / ".venv"
    interpreter = target / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
    if not interpreter.exists():
        # Keep dependencies isolated from the user's global Python install.
        print("Creating project environment:", target, flush=True)
        venv.EnvBuilder(with_pip=True).create(target)
    else:
        print("Reusing project environment:", target, flush=True)
    run([interpreter, "-m", "pip", "install", "--upgrade", "pip"])
    available = subprocess.run(
        [str(interpreter), "-c", "import torch, torchvision; print('Existing PyTorch:', torch.__version__)"],
        cwd=ROOT, capture_output=True, text=True,
    )
    if available.returncode == 0:
        # Reuse an existing working PyTorch installation, which may be a
        # hardware-specific build supplied by the developer.
        print(available.stdout.strip())
    else:
        # The default environment is deliberately the portable CPU build.
        run([interpreter, "-m", "pip", "install", "torch", "torchvision",
             "--index-url", "https://download.pytorch.org/whl/cpu"])
    run([interpreter, "-m", "pip", "install", "-r", ROOT / "requirements.txt"])
    run([interpreter, "-m", "pip", "check"])
    run([interpreter, ROOT / "scripts" / "check_install.py"])
    # Record the exact resolved environment for troubleshooting. These files
    # describe one machine; they are not a portable installation recipe.
    frozen = run([interpreter, "-m", "pip", "freeze"], capture_output=True, text=True)
    (ROOT / "requirements-local.txt").write_text(frozen.stdout, encoding="utf-8")
    details = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "platform": platform.platform(),
        "environment_python": str(interpreter),
        "note": "Imports verified. Camera and model inference must be checked separately.",
    }
    (ROOT / "environment-local.json").write_text(json.dumps(details, indent=2), encoding="utf-8")
    print("\nSetup complete. In VS Code, press F5 and choose Rover: webcam detection.")
    print("The first detection or inference check downloads the pretrained model.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"\nSetup failed: {exc}", file=sys.stderr)
        print("Read the error above; check Python, internet access, and README.md.", file=sys.stderr)
        raise SystemExit(1)
