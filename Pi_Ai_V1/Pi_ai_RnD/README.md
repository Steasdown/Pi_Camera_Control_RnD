# Pi AI Camera RnD — Stage 1 (Path 3)

Stage 1 target (later commits):
- Custom Picamera2 + IMX500 app scaffold (people-only overlay, bbox output).
- **This commit is templates only** (no camera access).

## Repo layout
- `src/pi_ai_rnd/` : Python package (future app code)
- `scripts/`      : Pi setup/run/smoke-test scripts
- `configs/`      : Reference configs (rpicam postprocess JSON, etc.)
- `docs/`         : Notes and plan checkpoints

## Quick start (Pi)
```bash
cd ~/Pi_Camera_Control_RnD/Pi_ai_RnD
chmod +x scripts/*.sh
./scripts/smoke_test.sh
./scripts/pi_setup.sh
./scripts/run.sh
```
