# rclnodejs 2.0.0 Release Roadmap

Working notes for cutting **rclnodejs 2.0.0** on the `lyrical` branch,
alongside the **ROS 2 Lyrical Luth GA** (May 22, 2026). Tracks issue
[#1458](https://github.com/RobotWebTools/rclnodejs/issues/1458) plus the
release-readiness gaps surfaced during the May 15 audit.

## 1. Branch state

- `lyrical` is currently at the same commit as `origin/develop`
  (`128faf6`, "Align WS and HTTP transports on a default `/capability`
  path (#1515)"). No divergence.
- `package.json` version: `2.0.0-beta.0` (not bumped yet).
- Web runtime tests already in place:
  [test/test-runtime.js](../test/test-runtime.js),
  [test/test-web-cli.js](../test/test-web-cli.js),
  [test/test-web-http.js](../test/test-web-http.js),
  [test/test-web-ws.js](../test/test-web-ws.js).

## 2. Roadmap status (#1458)

### Completed and shipped in 2.0.0-beta.0

- [x] Add `lyrical` to distro detection / runtime mapping
      ([#1485](https://github.com/RobotWebTools/rclnodejs/pull/1485),
      [#1488](https://github.com/RobotWebTools/rclnodejs/pull/1488))
- [x] Raise Node.js minimum to 20.20.2
      ([#1478](https://github.com/RobotWebTools/rclnodejs/pull/1478))
- [x] Gate C++20 by ROS distro
      ([#1477](https://github.com/RobotWebTools/rclnodejs/pull/1477))
- [x] Remove legacy compatibility code
      ([#1479](https://github.com/RobotWebTools/rclnodejs/pull/1479))
- [x] Refresh Electron prebuild target + demo apps
      ([#1492](https://github.com/RobotWebTools/rclnodejs/pull/1492))
- [x] Migrate Rolling build to 26.04
      ([#1493](https://github.com/RobotWebTools/rclnodejs/pull/1493))
- [x] Add Lyrical CI lanes (Linux x64, arm64, Windows)
      ([#1496](https://github.com/RobotWebTools/rclnodejs/pull/1496))
- [x] Pump CI to Node.js 26.x
      ([#1497](https://github.com/RobotWebTools/rclnodejs/pull/1497))
- [x] Add Lyrical lane to Linux prebuild workflows
      ([#1498](https://github.com/RobotWebTools/rclnodejs/pull/1498))
- [x] Tag `2.0.0-beta.0`
      ([#1499](https://github.com/RobotWebTools/rclnodejs/pull/1499))
- [x] Run Windows tests on Lyrical instead of Jazzy
      ([#1506](https://github.com/RobotWebTools/rclnodejs/pull/1506))
- [x] Surface Lyrical in README, BUILDING.md, npm landing page
      ([#1507](https://github.com/RobotWebTools/rclnodejs/pull/1507),
      [#1508](https://github.com/RobotWebTools/rclnodejs/pull/1508))
- [x] `rclnodejs/web` capability runtime over WebSocket
      ([#1509](https://github.com/RobotWebTools/rclnodejs/pull/1509),
      [#1511](https://github.com/RobotWebTools/rclnodejs/pull/1511))
- [x] HTTP transport for `call` / `publish`
      ([#1512](https://github.com/RobotWebTools/rclnodejs/pull/1512))
- [x] `rclnodejs-web` CLI launcher
      ([#1513](https://github.com/RobotWebTools/rclnodejs/pull/1513))
- [x] Default `/capability` path alignment
      ([#1515](https://github.com/RobotWebTools/rclnodejs/pull/1515))

### Open in #1458 — needed before GA

- [ ] Verify `npm pack` / `npm publish` includes Lyrical-tagged
      prebuilt binaries.
- [ ] Update TypeScript declarations if Lyrical introduces new message
      types or API changes (run `npm run generate-tsd-messages` on a
      Lyrical install and diff `types/`).
- [ ] Define support levels in user-facing docs — clarify what each
      tier means for CI coverage. Roadmap text:
      Full → Lyrical / Jazzy / Rolling, Transitional → Kilted,
      Compatibility → Humble.
- [ ] Draft a 1.x → 2.x migration guide (`docs/MIGRATION_2.x.md`).
- [ ] Validate newer graph + service-endpoint APIs on Lyrical
      (`getClientsInfoByService`, `getServersInfoByService`,
      `isContentFilterSupported`).

### Lower priority / post-GA

- [ ] Review `rmw_zenoh_cpp` support (Tier 1 in Lyrical, not default).
- [ ] Verify `rcl_logging_implementation` runtime backend selection.
- [ ] Add Windows Lyrical coverage once upstream artifacts stabilize.

## 3. PRs to land before GA

Both target `develop` and have **no file overlap**, so they can land in
either order with no conflict.

| PR                                                                                                                           | Branch                                 | Files                        | Notes                                                                                                                             |
| ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------- | ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| [#1514](https://github.com/RobotWebTools/rclnodejs/pull/1514) `[Web Runtime] rclnodejs/web SDK guide + JS/TS demos`          | `minggang/feat/web-tutorial-and-demos` | 26 (+1974/-160)              | Docs + demos + light CLI/runtime touch-ups. CI green. 3 Copilot review threads outstanding.                                       |
| [#1516](https://github.com/RobotWebTools/rclnodejs/pull/1516) `[Lyrical] Install from apt-deb repo instead of dated tarball` | `minggang/lyrical-apt-install`         | 4 GH-Actions YAMLs (+26/-49) | Replaces dated Lyrical tarball with `apt-get install ros-lyrical-desktop`. CI partially passing — confirm failures are unrelated. |

### Landing strategy (recommended)

Keep `lyrical` strictly equal to `develop` until the GA cut: merge each
PR to `develop` first, then fast-forward `lyrical`.

```bash
# After both PRs merge to develop on GitHub:
git fetch origin
git checkout lyrical
git merge --ff-only origin/develop
git push origin lyrical
```

Avoid cherry-picking the PR tips onto `lyrical` directly — it creates
rewritten commits that won't share SHAs with `develop` and makes future
merges noisy.

## 4. Release-readiness gaps not in #1458

- [ ] **Version bump.** `package.json` is still `2.0.0-beta.0` — needs
      `2.0.0` (or `2.0.0-rc.0` if you want one more candidate gate).
- [ ] **No `CHANGELOG.md`.** [docs/DISCOURSE_2.0.0.md](DISCOURSE_2.0.0.md)
      is an announcement post, not a changelog. Either add a real
      `CHANGELOG.md` or use the GitHub Release body. Either way, list
      breaking changes explicitly.
- [x] **Decide on the untracked planning docs in `docs/`:** Done.
  - `DISCOURSE_2.0.0.md` — polished announcement; commit before posting.
  - `DISCOURSE_2.0.0-beta.0.md` — historical record of beta.0 announcement; commit.
  - `RCLNODEJS_WEB_VS_ROSBRIDGE.md`, `WEB_RUNTIME_ARCH.md`,
    `WEB_RUNTIME_ROADMAP.md` — internal planning; version-label scrub
    applied (2.1→2.0, 2.2→2.1, 2.3→2.2, 2.4→2.3); commit as repo history.
  - `DISCOURSE_2.1.0.md` — deleted (superseded by `DISCOURSE_2.0.0.md`).
  - `CAPE_NAMING.md` — deleted (self-marked SUPERSEDED; brand reverted).
- [ ] **npm dist-tag plan.** Today `2.0.0-beta.0` is on `next`. Decide:
      tag `2.0.0` as `latest` immediately, or hold on `next` for a soak
      period?
- [ ] **Demo `package.json` dependency form.** Per Copilot review on
      #1514, [demo/web/typescript/package.json](../demo/web/typescript/package.json)
      points at `github:minggangw/...#feat/web-tutorial-and-demos`.
      After GA, change to `"rclnodejs": "^2.0.0"` so the demo
      `npm install`-s reproducibly against the published package.
- [ ] **`exports` map smoke test.** The `./web/server` entry maps to
      `./lib/runtime/index.js`. Add a CI step that runs
      `node -e "require('rclnodejs/web/server')"` against a packed tarball.
- [ ] **Post-GA tracking issue.** Spin off the "Lower priority / post-GA"
      items in #1458 into a fresh tracking issue (or 2.1.0 milestone) so
      they aren't lost when #1458 closes.

## 5. Suggested action order

1. Resolve open Copilot review threads on
   [#1514](https://github.com/RobotWebTools/rclnodejs/pull/1514), then
   merge to `develop`.
2. Merge [#1516](https://github.com/RobotWebTools/rclnodejs/pull/1516)
   to `develop` (independent of #1514).
3. Fast-forward `lyrical` to `develop`.
4. On `lyrical`:
   - Bump version to `2.0.0`.
   - Add `CHANGELOG.md` and `docs/MIGRATION_2.x.md`.
   - Add the support-policy paragraph to README / BUILDING.
   - Decide on the 7 untracked `docs/*.md` files.
   - Run `npm pack --dry-run` and grep for `lyrical-*.node` in the file
     list to confirm prebuilds are bundled.
5. Tag `2.0.0` on `lyrical`, publish to npm, post the discourse
   announcement, then close #1458 and open a fresh tracking issue for
   the post-GA items.

## 6. References

- Tracking issue: [#1458](https://github.com/RobotWebTools/rclnodejs/issues/1458)
- Lyrical install guide: <https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html>
- Lyrical timeline: <https://docs.ros.org/en/rolling/Releases/Release-Lyrical-Luth.html>
- Discourse announcement draft: [docs/DISCOURSE_2.0.0.md](DISCOURSE_2.0.0.md)
