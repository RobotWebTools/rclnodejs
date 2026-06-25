# JSDoc Workflow

This directory contains the custom JSDoc template, the landing-page generator,
and the staging script used to prepare the docs content that is published to
GitHub Pages.

The published version set is curated in
[`published-versions.json`](./published-versions.json). That manifest is the
source of truth for which versions appear on the live docs site. Each staging
run rebuilds the whole site from scratch: every listed version is rebuilt from
its Git tag. The staged tree is therefore a pure function of the manifest plus
the tags — there is no `gh-pages` branch state to maintain or drift out of sync.

## Commands

### `npm run docs`

Build local docs for the current workspace version.

Output:

- `docs/<current-version>/`
- `docs/index.html`

Use this to verify the docs for the version currently declared in
`package.json`.

### `npm run docs:publish`

Stage the full publishable docs tree under `build/published-docs/`.

Behavior:

- reads the published version set from `published-versions.json`
- rebuilds every listed version from its Git tag
- rebuilds the staged landing page index

This is the normal command to use for a new release, and it is fully
deterministic: deleting `build/` and rerunning reproduces the identical tree.

This does **not** rebuild docs for every historical `rclnodejs` tag — only the
curated subset listed in the manifest. To change which versions are published,
edit `published-versions.json`.

The script reads the manifest next to it by default. Override the inputs only
for testing:

- `--manifest <path>` — use a different version manifest
- `--out <dir>` — stage into a different directory

## New Release Example

For a new release such as `1.9.0`:

1. Update `package.json` to `1.9.0`.
2. Add `1.9.0` to the `versions` array in `published-versions.json`.
3. Run `npm run docs` to preview the new version locally (`docs/1.9.0/` and
   `docs/index.html`).
4. Cut and push the `1.9.0` release tag so `npm run docs:publish` can rebuild
   it from Git.
5. Run `npm run docs:publish`.
6. Verify the staged output in:
   - `build/published-docs/docs/1.9.0/`
   - `build/published-docs/docs/index.html`
   - `build/published-docs/.nojekyll`
7. Publish the contents of `build/published-docs/` to GitHub Pages (the
   `deploy-docs.yml` workflow does this automatically on a `docs-*` tag).

## GitHub Actions Deployment

The `deploy-docs.yml` workflow (`.github/workflows/deploy-docs.yml`) automates
building and deploying docs to GitHub Pages.

### Triggers

- **Tag push** matching `docs-*` (e.g. `git tag docs-1.9.0 && git push origin docs-1.9.0`) — builds and deploys automatically.
- **Manual dispatch** from the Actions tab — includes a `dry_run` toggle
  (defaults to `true`). Set it to `false` to deploy.

These workflow runs only execute in the upstream `RobotWebTools/rclnodejs`
repository. If you trigger the workflow from a fork, the build job is skipped,
so manual dispatches and `docs-*` tag pushes there will not run the docs build.

### What it does

1. Full checkout with all tags.
2. Runs `npm run docs:publish` to stage the docs tree. This reads
   `published-versions.json` and rebuilds every listed version from its tag.
3. Uploads the staged output as a Pages artifact.
4. Deploys to GitHub Pages (skipped when `dry_run` is `true`).

The workflow only needs `contents: read` and never writes to any branch.
Because the manifest plus the Git tags fully describe the published set, the
live site is always reconstructed from a fresh build — nothing can silently
drop off.

### Testing

- Run the workflow manually with `dry_run` enabled first to verify the build
  succeeds without deploying.
- Only push a `docs-test` tag when it is acceptable to update the published
  GitHub Pages site: `docs-*` tags perform a real production deploy.
  Afterward, clean up the tag with:
  `git tag -d docs-test && git push origin :refs/tags/docs-test`.

## Manual Landing Page Rebuild

If the staged docs tree already exists and you only want to rebuild
`build/published-docs/docs/index.html`, run `tools/jsdoc/build-index.js` against
that docs root and point it at the package metadata for the latest published
version.

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

## Notes

- The staged publish output keeps shared assets in `build/published-docs/docs/_static/`.
- `.nojekyll` must remain in the staged output because the published docs tree
  uses an underscore-prefixed directory.
- `published-versions.json` is the source of truth for which versions are
  published; the Git tags are the content source.
