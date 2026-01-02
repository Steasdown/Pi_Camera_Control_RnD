# Stage 1 — Living Notes (Path 3)

This is a living checklist. Update as you validate on the Pi.

## Checkpoints
- [x] 4.1 Template files created
- [ ] 4.2 Pi validates structure only (smoke_test.sh)
- [ ] 4.3 Prototype A: open camera + print tensor shapes once
- [ ] Prototype B: people-only + bbox prints (rate limited)
- [ ] Prototype C: red overlay on preview
- [ ] Prototype D: CSV logging + stable runtime config

## Rules
- Pi-only operations live in `scripts/`
- IMX500 parsing lives in `src/pi_ai_rnd/imx500_adapter.py`
- Drawing lives in `src/pi_ai_rnd/overlay.py`
- SSH-friendly debug output; tee logs to `logs/run.log`

## Test log template
Date:
Commit:
What changed:
Pi result:
Notes:
