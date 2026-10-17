# vision_bot v2: Handoff to Claude Code (run on the PC)

Read, in order: `docs/superpowers/specs/2026-10-08-vision-bot-v2-nav-design.md` (binding spec), `docs/superpowers/plans/2026-10-08-vision-bot-v2-nav.md` (the plan), `docs/V2_DISCOVERY.md` (static findings), `docs/DECISIONS.md` (approved rulings), `docs/V2_RUNBOOK.md` (pitfall table).

State: Task 0 is partly done. `docs/V2_DISCOVERY.md` is written from reading the repos; the live checks, dependency install (rebuild the Docker image), `scripts/check_v1.sh`, and the clean `colcon build` baseline are not done. Nothing is committed yet; the docs are untracked files on `main`. Create branch `v2/task-0-discovery` before committing.

Rules that override the executing-plans skill's "keep going" default: the plan's Global Constraints require **stopping at every gate** with a report and waiting for Zach's go. Append to `docs/INTERVIEW_NOTES.md` at every gate.

Run everything sim-related inside the container: `cd docker && docker compose run --rm ros` (or `exec ros bash` in a second terminal). The Gazebo GUI needs VcXsrv; use `gui:=false` with `xvfb-run` for headless checks. Host ports are 9091 (rosbridge) and 8081 (video).

Kickoff prompt: "Read docs/V2_HANDOFF.md and execute the plan at docs/superpowers/plans/2026-10-08-vision-bot-v2-nav.md starting at Task 0. Stop at every gate and report."
