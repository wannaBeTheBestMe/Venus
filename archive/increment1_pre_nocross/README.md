# Increment 1 — pre-no-cross snapshot

These files implement the **Increment-1 plan** (detection + corner-finding + mapping
mechanics). They are a frozen snapshot taken before the Increment-2 redesign.

⚠️ **NOT cliff-safe.** The FOLLOW state in `boundary.h` traces a zigzag that
*straddles / crosses* the black tape. That is fine for bench-validating detection
and mapping on flat tape, but on the real arena the boundary is a cliff and
crossing it counts as falling off (mission loss). Increment 2 replaces FOLLOW with
an inside-only edge-hug and adds an ACQUIRE phase + forward-obstacle avoidance.

- Full Increment-1 spec:  `~/.claude/plans/increment1-pre-nocross.md`
- Active (Increment-2) plan: `~/.claude/plans/can-you-analyze-the-noble-book.md`
- Git restore point: tag `increment1-pre-nocross`

Files: `boundary.h`, `main.c`, `UI_code.py` (verbatim copies of the live files at
snapshot time). Do not edit these — they are the historical record.
