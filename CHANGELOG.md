# Changelog

## 1.0 (2026-10-18)
- Clamp the body-contact time of impact to 1 (vehicles could move many times their velocity into each other).
- Repair a garbage contact point instead of the "FATAL dynamic error ... MI.col.p" crash.
- Fix vehicles wedging, freezing and merging on slow contact: touching contacts no longer cancel
  sliding or separating motion, the into-surface part of the motion is removed instead of the
  whole step, and overlapping bodies are pushed apart by mass share.
