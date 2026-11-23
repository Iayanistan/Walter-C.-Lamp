# Installs the CUDA build of PyTorch for the RTX A1000 and the project dependencies,
# then verifies that torch can see the GPU.
$ErrorActionPreference = "Stop"

Write-Host "==> Installing CUDA (cu128) build of torch/torchvision..."
pip install --force-reinstall --no-deps torch==2.10.0+cu128 torchvision==0.25.0+cu128 --index-url https://download.pytorch.org/whl/cu128

Write-Host "==> Installing project requirements..."
pip install -r (Join-Path $PSScriptRoot "..\requirements.txt")

Write-Host "==> Verifying GPU visibility..."
python -c "import torch; ok = torch.cuda.is_available(); print('cuda available:', ok); print('device:', torch.cuda.get_device_name(0) if ok else 'none'); raise SystemExit(0 if ok else 1)"
