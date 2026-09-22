# SDD ledger — plan: docs/superpowers/plans/2026-09-22-hd-utility-ddev-eight-channel.md

Setup: baseline command `$env:PYTHONPATH='src'; python -m pytest -q` -> 31 passed.

Ruling: repository has no `.git`; isolated git worktree, task commits, task-start/task-done scripts and commit-range review are unavailable. Work remains confined to the explicitly named project directory, and evidence is recorded in this ledger and generated reports. Cost if wrong: no atomic git rollback; destructive cleanup therefore occurs only after exact target inventory and after replacement artifacts pass validation.

Pre-flight: Task 1 produces the transformed `run_all.par` consumed by Tasks 3-6; Task 2 produces the 8-channel contract consumed by Tasks 3-7; Task 3 produces `simfile.sim` consumed by Tasks 4-6. The interfaces agree on wheel order FL,FR,RL,RR, imports 8, exports 16.

Ruling: the approved plan names `tests/test_channels.py`, but the existing suite uses `tests/test_channel_contract.py`; modify the existing file to preserve one authoritative contract test. Cost if wrong: none beyond file naming.

Ruling: the user added cleanup, single-wheel pothole construction, one real run and video export after plan approval; execute them as Tasks 9-11 after the eight planned tasks. Cost if wrong: later tasks may expose visualization constraints requiring a documented adjustment.
