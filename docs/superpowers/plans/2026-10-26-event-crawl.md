# Event Crawl Implementation Plan

2026-10-27进度：Task1–3完成；Task4完成有限覆盖与失败归因，10cm宽坑仍未达标。用户检查后要求缩短场景距离、简短收尾并暂停调优；两条缩短距离工况均通过。Task5完成本轮检查点交付，不标记整个深坑策略开发完成，不再继续启动调参。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline. User explicitly authorized autonomous execution of the agreed design and tuning; do not pause for plan approval again. Fresh-context review at final delivery.

**Goal:** Develop classic staged, speed-adaptive shallow-pit crossing, prioritizing comfort and reliable traction rather than constant pit speed.

**Architecture:** Separate expert-two event controller, per-wheel sensor-gated phases, vehicle speed envelope and bounded stage force/torque coordination. Legacy controller and assessment remain available; worker chooses explicit policy and captures every dependency.

**Tech Stack:** Python3.8/numpy/pytest; TruckSim2019/MF52-3114; maximum2 isolated native processes.

**Spec:** ../specs/2026-10-26-event-crawl-design.md

## Global Constraints

- SI FL/FR/RL/RR; same pit/vehicle/model identity and expert-one isolation.
- ±500Nm/1000Nm/s; ±18200N/45400N/s; initial working force6000N. No pose truncation; full native duration.
- Legacy default commands and criteria unchanged. New event_crawl_v1 criteria are explicit, not historical replacements.
- Freeze source/config per experiment, unique run IDs, preserve native playback; no exported video.
- Missing real chassis/actuator data and tire-model limits remain explicit; simulation acceptance is never real-vehicle acceptance.

## Review Focus

- Single-side left wheels never acquire pit phases; front exit alone cannot complete rear crossing.
- Lost/recontact hysteresis and stale feedback do not cause phase chatter or sudden drive restoration.
- Lower speed does not mean permanent stop; stop completion requires actual rear exit and dwell.
- Integral/force/torque projection obey actual capability; integrator step refinement never changes controller periods.
- New policy/source provenance and task version prevent mixed-config PASS or legacy-result rewrites.

## Task1 — phase recognition and speed envelope

Files: create event_config.py, event_phases.py, event_speed.py; tests/test_event_phases.py and test_event_speed.py under tests/shallow_press/.
Interfaces: EventCrawlConfig frozen dataclass; WheelPhaseTracker.update(t,x4,y4,fz4,cmp4,scene)->dict with phases/base_phases/contact/expected; EventSpeedPlanner.step(dt,requested_m_s,ceiling_m_s)->float; speed_ceiling(x4,expected,scene,config)->float.
- [x] RED tests: correct tracks, both axle sequence, lost/recontact dwell, backward movement; braking before all four edges; jerk/acceleration bounds and instance reset.
- [x] Implement minimal causal geometry/contact classification and explicit bounds; run focused tests GREEN; commit.

## Task2 — event controller and execution integration

Files: event_controller.py; config.py policy/event settings; worker/native/actuation explicit selection and optional stage context; tests/test_event_control.py.
Interfaces: EventCrawlStrategy reset/step/status/summary same contract; motor_context optional per-wheel phase/speed target for execution, legacy None. Existing public nine commands unchanged.
- [x] RED tests: initial stationary leveling, stage feedback signals, same capability/slew, stale unloading, finite completion, separate instances; legacy native bridge compatibility.
- [x] Implement speed-only baseline first, then phase force/torque hooks with defaults; freeze every new runtime file. Full suite GREEN; commit.

## Task3 — explicit event assessment and provenance

Files: event_assessment.py plus worker result/log/summary, analysis support; tests/test_event_assessment.py.
Interfaces: assess_event(values,scene,summary,config,diagnostics)->result; old assess output retained under legacy_assessment. New criteria exclude only constant-mean speed tracking, add event reference/overspeed and progress; no other old thresholds relaxed.
- [x] RED tests: never-entered/only-front/stalled/overspeed cannot pass; a benign variable reference is not failed merely for low mean; model/capability scans never baseline acceptance.
- [x] Implement binding of full raw evidence/diagnostics and explicit task version. Full suite GREEN; commit.

## Task4 — deep-first development experiments

Files: configs/shallow_press/event_crawl_20261026.json, versioned evidence/shallow_event_crawl_20261026/ cohorts and report.
- [x] Freeze and run matched 10cm wide/right low-speed anchors; complete durations60–120s as needed, max2 workers.
- [x] Read phase force/torque/contact/speed at failures before each change. Isolate speed-only versus full suspension and traction-only under same speed planner.
- [ ] Select candidate only on joint criteria; validate half-step, delay, neighboring depth/speed. Expand requested approach speeds only after deep anchors improve; record actual pit slowdown.
- [x] Archive unsuccessful options with source hashes; keep only useful active interfaces and explicit research defaults.

Paused by user after checkpoint: 4cm two-track discrete approach1/2/3/4/5/6 points pass;6cm right-only3km/h point passes; wide6cm and8–10cm are not qualified. Half-step4cm both pass,80ms body-delaywide fails. Final scene-only comparison shrinks3km/h12→8m,6km/h30→22m, unchanged controller, bothwide4cm full80s pass. Unexecuted climb coordination prototypes archived, not production features.

## Task5 — final review and delivery

- [x] Fresh full tests, diff check, expert-one/profile/legacy regression; source/input/raw/model/native-history hashes and frozen bytes.
- [x] Fresh-context final review; fix important issues with RED→GREEN, preserve evidence versions.
- [x] Commit report, useful controller and evidence; tag milestone, leave unrelated user files untouched. Report validated envelope and unmet goals candidly.

## Ledger

- Ruling: execute in existing feature checkout and branch, not main, with backup tag; preserves ignored native histories and the user's current-project preference.
- Ruling: initial event policy uses explicit conservative edge targets to be tuned; they are simulation assumptions, not proven limits. New successful slowdown missions do not claim constant-speed capability.
