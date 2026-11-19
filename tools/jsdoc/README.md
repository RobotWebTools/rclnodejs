# JSDoc Workflow

Templates and scripts for building API docs and publishing them to GitHub Pages.

## Commands

Run from the repository root:

| Command | Output |
| --- | --- |
| `npm run docs` | Current package version in `docs/<version>/`, plus `docs/index.html` |
| `npm run docs:publish` | Full publication tree in `build/published-docs/` |

`docs:publish` builds the versions in [`published-versions.json`](./published-versions.json) using temporary worktrees, then rebuilds the index and shared assets. It leaves tags unchanged and needs no previous site output or `gh-pages` branch.

To test another manifest or output directory, pass `--manifest <path>` or `--out <dir>` after `npm run docs:publish --`.

## Documentation Source Tags

For each version:

1. Use the exact `docs-<version>` tag if it exists.
2. Otherwise use the exact `<version>` release tag.
3. Fail if neither tag exists or the selected source declares a different version in `package.json`.

Keep release tags unchanged and documentation commits API-compatible with the release. For example, `docs-2.3.0` can correct docs without changing `2.3.0`. Logs show the selected tag. A broken documentation tag fails the build; it does not fall back to the release tag.

Create or fetch source tags before `npm run docs:publish`. Checking out a corrected branch does not change the selected source.

## New Releases

1. Update `package.json` and add the version to `published-versions.json`.
2. Run `npm run docs` to preview the current version locally.
3. Create the release tag. If using corrected documentation, create a `docs-<version>` tag at the documentation commit too.
4. Run `npm run docs:publish` and inspect `build/published-docs/docs/<version>/`, `build/published-docs/docs/index.html`, and `build/published-docs/.nojekyll`.
5. Merge tooling and manifest changes into `develop` before pushing a `docs-*` tag. Alternatively, dispatch the deployment workflow from the validated tooling branch with the source tags available remotely.

## GitHub Actions Deployment

The [deployment workflow](../../.github/workflows/deploy-docs.yml) uses Node.js 24.

| Trigger | Tooling source | Deployment |
| --- | --- | --- |
| `docs-*` tag push | `develop` | Always deploys the full manifest |
| Manual dispatch | Selected commit (`github.sha`) | `dry_run` defaults to `true`; set it to `false` to deploy |

Pushing a `docs-*` tag, including a test tag, updates the live site. Validate locally first, then use manual dispatch with `dry_run: true` for a remote build without deployment.

GitHub reads the workflow from the triggering ref before checkout. Old documentation tags may lack the checkout rule above; use manual dispatch from an updated branch for those tags.

The workflow fetches full history and tags, runs `npm run docs:publish`, uploads a Pages artifact, and deploys unless `dry_run` is enabled. It never writes to a branch. Builds run only in `RobotWebTools/rclnodejs`; forks are skipped.

### Pull Request and Push Validation

The [Linux x64 build/test workflow](../../.github/workflows/linux-x64-build-and-test.yml) runs `npm run docs` in its Lyrical / Node 26 lane for pull requests and pushes. It checks the current API docs and landing page; JSDoc errors fail the job. It does not build historical tags, upload artifacts, or deploy. Use a publication dry run to check the tagged versions.

## Manual Landing Page Rebuild

To rebuild only `build/published-docs/docs/index.html`, point `tools/jsdoc/build-index.js` at an existing staged tree and the package metadata for its latest published version.

Example for published version `1.8.0`:

```bash
mkdir -p build/published-docs/.tmp
git show 1.8.0:package.json > build/published-docs/.tmp/package-1.8.0.json

export RCLNODEJS_DOCS_ROOT="$PWD/build/published-docs/docs"
export RCLNODEJS_DOCS_INDEX_PATH="$PWD/build/published-docs/docs/index.html"
export RCLNODEJS_LOCAL_INDEX_PATH=''
export RCLNODEJS_PACKAGE_JSON_PATH="$PWD/build/published-docs/.tmp/package-1.8.0.json"

node tools/jsdoc/build-index.js
rm -rf build/published-docs/.tmp
```

Keep `build/published-docs/.nojekyll` in the output so Pages serves the shared assets under `docs/_static/`.
