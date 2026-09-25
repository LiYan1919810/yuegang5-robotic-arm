---
name: robotic-arm
description: Robotic arm algorithm control development skill for team collaboration. This skill should be used when a team collaboratively implements robotic arm algorithm control modules (kinematics, trajectory planning, motion control, etc.), needs lightweight version management and development records (dev logs, changelogs), or needs branching and other Git-based development operations when required.
agent_created: true
---

# Robotic Arm

## Overview

This skill enables a team (or multiple agents) to collaboratively implement the algorithm control part of a robotic arm — e.g. forward/inverse kinematics, trajectory planning, servo control loops, coordinate transforms — with lightweight version management and persistent development records. It defines a shared directory layout, a collaboration workflow, and conventions so that every change is traceable and every decision is documented.

## Directory Layout

Work inside the project's robotic-arm module directory. Establish it on first use if it does not exist:

```
robotic-arm/
├── src/                  # Algorithm source code
│   ├── kinematics.py     # Forward/inverse kinematics
│   ├── trajectory.py     # Trajectory planning
│   └── control.py        # Servo/control loops
├── tests/                # Unit and integration tests
├── docs/
│   ├── DEVLOG.md         # Development records (append-only, one entry per session)
│   └── CHANGELOG.md      # Version history (Keep a Changelog format)
└── README.md             # Module intro, build & run instructions
```

Adapt file names to the actual language and framework chosen by the team; keep the `docs/DEVLOG.md` and `docs/CHANGELOG.md` conventions unchanged, because all collaborators rely on them.

## Collaboration Workflow

### Step 1: Read the current state first

Before writing any code:

1. Read `docs/DEVLOG.md` from the bottom (most recent entries) to understand the latest progress, decisions, and open issues.
2. Read `docs/CHANGELOG.md` to know the current version number and what changed in it.
3. If the repository is Git-based, run `git status`, `git log --oneline -10`, and check active branches with `git branch -a` to understand where development stands.

Never overwrite or contradict a decision recorded in DEVLOG without first noting the reason for the change.

### Step 2: Decide whether a branch is needed

- Work directly on `main` only for small fixes (docs, comments, bugfixes under ~20 lines).
- Create a feature branch for any new algorithm module, interface change, or risky experiment:

```bash
git checkout -b feature/<short-topic>   # e.g. feature/inverse-kinematics
```

- Name branches `feature/<topic>`, `fix/<topic>`, or `experiment/<topic>`. Use lowercase kebab-case.
- When a branch's work is verified by tests and recorded in DEVLOG/CHANGELOG, merge it back to `main` and delete the branch.

If Git is not initialized (early-stage local development), skip branching but still follow the dev record conventions below.

### Step 3: Implement with team conventions

- Keep each algorithm function small, pure where possible, and unit-testable. Kinematics and control functions must be deterministic and numerically documented (input units: radians/meters/seconds; output frames clearly named).
- Every new public function needs a corresponding test in `tests/` before being merged.
- Prefer well-known formulations (DH parameters, Jacobian-based IK, PID/computed-torque control) and cite the convention used in the docstring.

### Step 4: Record development (mandatory, every session)

1. Append an entry to `docs/DEVLOG.md` (newest at the bottom, append-only):

```markdown
## 2026-09-25 — <contributor/agent name> — <topic>
- Done: implemented Jacobian-based IK for 6-DOF arm, 12 unit tests passing
- Decision: chose damped least squares over analytical IK because the arm has a spherical wrist but joint limits make analytical solutions brittle
- Next: add singularity detection before merging trajectory module
```

2. Update `docs/CHANGELOG.md` when a user-visible change or version milestone is reached:

```markdown
## [0.2.0] - 2026-09-25
### Added
- Inverse kinematics module with damped least squares solver
### Fixed
- DH parameter sign error in joint 4 transform
```

3. If Git is in use, commit with a concise imperative message (`git commit -m "Add damped least squares IK solver"`), one logical change per commit.

### Step 5: Version numbering

Use semantic versioning on the module: `MAJOR.MINOR.PATCH`. Bump MINOR for new algorithm capabilities, PATCH for fixes, MAJOR for breaking interface changes. Record each bump in CHANGELOG.

## When to Use This Skill

- A team asks to design or implement robotic arm algorithm control (kinematics, planning, control).
- Multiple agents/contributors need to continue work on the same robotic-arm codebase.
- A request mentions dev records, version management, branching, or merge workflows for this module.
