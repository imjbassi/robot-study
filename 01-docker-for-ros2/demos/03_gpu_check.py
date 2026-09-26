"""Check whether this container can see an NVIDIA GPU (i.e. was started with --gpus all).

    python3 03_gpu_check.py

Checks three layers, from lowest to highest:
  1. Device nodes   -- /dev/nvidia0, /dev/nvidiactl (injected by the NVIDIA Container Toolkit)
  2. Driver         -- nvidia-smi (also injected by the toolkit)
  3. Frameworks     -- PyTorch / JAX, if installed
"""

import glob
import shutil
import subprocess


def check_device_nodes():
    nodes = sorted(glob.glob("/dev/nvidia*"))
    print(f"[1] device nodes: {', '.join(nodes) if nodes else 'none'}")
    return bool(nodes)


def check_nvidia_smi():
    if not shutil.which("nvidia-smi"):
        print("[2] nvidia-smi: not found")
        return False
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        print(f"[2] nvidia-smi: failed: {out.stderr.strip()}")
        return False
    for line in out.stdout.strip().splitlines():
        print(f"[2] nvidia-smi: {line}")
    return True


def check_frameworks():
    try:
        import torch
        ok = torch.cuda.is_available()
        name = torch.cuda.get_device_name(0) if ok else "no CUDA device"
        print(f"[3] torch {torch.__version__}: cuda available={ok} ({name})")
    except ImportError:
        print("[3] torch: not installed")

    try:
        import jax
        print(f"[3] jax {jax.__version__}: devices={jax.devices()}")
    except ImportError:
        print("[3] jax: not installed")


def main():
    has_nodes = check_device_nodes()
    has_driver = check_nvidia_smi()
    check_frameworks()
    if not (has_nodes and has_driver):
        print("\nNo GPU visible. Start the container with --gpus all and make sure the "
              "NVIDIA Container Toolkit is installed on the host.")


if __name__ == "__main__":
    main()
