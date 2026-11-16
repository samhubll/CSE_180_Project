# Codex kickoff prompt

Paste the following prompt into Codex with this repository open.

```text
Help me develop the CSE180 final project, Search the warehouse. Read AGENTS.md,
README.md, SETUP.md and ROADMAP.md first, then inspect the existing search and
detector packages. Preserve the assignment requirements and build on the
working setup rather than replacing it.

The end goal is a C++ search node that uses MRTP Navigator to explore an
unknown-person warehouse, localizes camera detections in map through TF2,
merges repeated observations, shows confirmed people in RViz, and returns to
its starting pose. Only the detector may be Python. All simulation-facing
nodes must use use_sim_time:=true.

For this first session, complete ROADMAP milestones 1 and 2: verify the live
sensor contract, then implement and test stationary detection-to-map
localization with confirmed-person markers. This is the first working slice;
the autonomous waypoint mission belongs to a later milestone.

Start with a short read-only preflight: inspect Git status, the starter code,
ROS topics, camera calibration, RGB/depth registration, depth encoding/units,
timestamps, optical-frame TF and the MRTP Navigator API. Read existing setup
evidence, but recheck assumptions needed by the implementation. Summarize
the approach briefly and proceed with the implementation.

Important details:
- The repository is /MRTP/CSE180/src/CSE_180_Project inside the CSE180
  workspace. Build using bash scripts/project.sh build, which enters the
  workspace and uses the correct Python environment. From Windows, use the
  configured Docker terminal/tasks or docker exec, not native PowerShell ROS.
- Detector inference is already implemented. Search currently only checks
  connections; its latest-depth pointer is not timestamp-safe localization.
- Verify camera/depth registration and calibration before indexing depth
  with RGB boxes. Use a bounded depth history, match the detection's source
  image time within a documented tolerance, and reject unmatched/stale data.
- Take a robust median of valid depth in a clipped central box region with
  enough support. Handle invalid values and the observed depth units.
- Use X=(u-cx)*d/fx, Y=(v-cy)*d/fy, Z=d in the optical frame. Transform to
  map at the image timestamp, never silently using latest TF for localization.
- Merge observations within 0.6 m, update position estimates, count distinct
  source images, and require at least two observations before confirming a
  person. Publish stable-ID markers in map.
- Keep known person positions exclusively in validation notes. Do not make
  detection, waypoints or expected counts depend on those positions.

Add meaningful C++ checks for geometry and bad inputs, then build and run
the detector/search with live Gazebo data. Verify header/time matching,
finite map points, confirmation and stable markers. Test with a visible
stationary person, then controlled turning to check timestamp handling.
Keep all movement confined to the Gazebo simulation. Save actual evidence;
do not invent detection ranges, successful tests or accuracy numbers.

Keep the code understandable for a student demonstration. Make routine,
reversible implementation decisions yourself and continue through the
acceptance checks. If credentials or missing information block a step,
explain exactly what is missing and continue useful unblocked work.

Finish by updating ROADMAP.md with completed and unverified items. Summarize
the changed files, commands and observed results, any remaining issues, and
the next milestone. Keep the changes local and reviewable; publishing remote
changes requires a separate instruction.
```

The prompt names the behavior, relevant files, constraints and verification,
following the official OpenAI guidance on
[prompting Codex](https://learn.chatgpt.com/docs/prompting).
