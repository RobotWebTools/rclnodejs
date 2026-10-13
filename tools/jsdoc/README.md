# JSDoc Workflow

This directory contains the custom JSDoc template, the landing-page generator,
and the staging script used to prepare the docs content that is published to
GitHub Pages.

The published version set is curated in
[`published-versions.json`](./published-versions.json). That manifest is the
source of truth for which versions appear on the live docs site. Each staging
run rebuilds the whole site from scratch, preferring `docs-<version>` for each
listed version and falling back to the `<version>` release tag when no matching
documentation tag exists. The staged tree is determined by the manifest, selected
tag commits and current documentation tooling — there is no `gh-pages` branch
state to maintain.

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
- selects `docs-<version>` when available, otherwise `<version>`
- checks that the selected source's package version matches the manifest entry
- rebuilds the selected sources in temporary worktrees without rewriting them
- rebuilds the staged landing page index

This is the normal command to use for a new release. Rebuilds use the same inputs
when the manifest, selected tag commits and documentation tooling are unchanged.

This does **not** rebuild docs for every historical `rclnodejs` tag — only the
curated subset listed in the manifest. To change which versions are published,
edit `published-versions.json`.

The script reads the manifest next to it by default. Override the inputs only
for testing:

- `--manifest <path>` — use a different version manifest
- `--out <dir>` — stage into a different directory

### Documentation Source Tags

For `docs-*` pushes, the workflow checks out `refs/heads/develop` for the
generator, version manifest and template. Manual dispatch uses the selected
ref's commit (`github.sha`), so tooling changes can be validated before merging.
The generator selects documentation sources independently for every version
in that manifest:

1. Use the exact `docs-<version>` tag if it exists.
2. Otherwise use the exact `<version>` release tag.
3. Fail if neither tag exists or the selected source declares a different
   version in `package.json`.

For example, `docs-2.3.0` can contain documentation corrections while `2.3.0`
remains unchanged. Older versions without documentation tags continue to use
their release tags. Logs identify the source selected for each version.
There are no release-specific source patches, and JSDoc errors remain fatal:
a broken documentation tag does not silently fall back to a release tag.

Keep release tags immutable and ensure documentation-tag commits remain
API-compatible with the associated release. Commit documentation corrections,
then create or update the matching documentation tag. The local generator must
see that tag to validate its source; selecting a fixed branch alone is not enough.
Pushing a `docs-*` tag triggers a real deployment, so validate locally first.
For a remote dry run, use manual dispatch with `dry_run: true` on a branch
containing the new tooling, with the desired source tags already available.

## New Release Example

For a new release such as `1.9.0`:

1. Update `package.json` to `1.9.0`.
2. Add `1.9.0` to the `versions` array in `published-versions.json`.
3. Run `npm run docs` to preview the new version locally (`docs/1.9.0/` and
   `docs/index.html`).
4. Cut and push the `1.9.0` release tag. If using a separate corrected
   documentation source, create a local `docs-1.9.0` tag at that commit.
5. Run `npm run docs:publish`.
6. Verify the staged output in:
   - `build/published-docs/docs/1.9.0/`
   - `build/published-docs/docs/index.html`
   - `build/published-docs/.nojekyll`
7. Merge tooling and manifest updates into `develop` before pushing a `docs-*`
   tag for automatic deployment. Alternatively, manually dispatch the workflow
   from the validated tooling branch to publish the site.

## GitHub Actions Deployment

The `deploy-docs.yml` workflow (`.github/workflows/deploy-docs.yml`) automates
building and deploying docs to GitHub Pages.

### Triggers

- **Tag push** matching `docs-*` (e.g. `git tag docs-1.9.0 && git push origin docs-1.9.0`) — builds and deploys automatically using tooling and the full version manifest from `develop`.
- **Manual dispatch** from the Actions tab — includes a `dry_run` toggle
  (defaults to `true`). Uses tooling from the selected commit, not automatically
  from `develop`. Set `dry_run` to `false` to deploy.

GitHub loads the workflow definition itself from the triggering ref before
checkout runs. A documentation tag must therefore contain this checkout rule
for automatic deployments to use it. For historical tags carrying an older
workflow, manually dispatch from a branch containing the updated workflow
instead of relying on the historical tag's deployment behavior.

These workflow runs only execute in the upstream `RobotWebTools/rclnodejs`
repository. If you trigger the workflow from a fork, the build job is skipped,
so manual dispatches and `docs-*` tag pushes there will not run the docs build.

### What it does

1. Check out deployment tooling from `develop` for tag pushes, or the selected
   commit for manual dispatch, with full history and all source tags.
2. Runs `npm run docs:publish` to stage the docs tree. This reads
   `published-versions.json` and rebuilds every listed version from its
   documentation tag or release-tag fallback.
3. Uploads the staged output as a Pages artifact.
4. Deploys to GitHub Pages (skipped when `dry_run` is `true`).

The workflow only needs `contents: read` and never writes to any branch.
The manifest, Git tags and current documentation tooling reconstruct the full
site from a fresh build — no prior deployment output is required.

### Pull Request and Push Validation

The [Linux x64 build/test workflow](../../.github/workflows/linux-x64-build-and-test.yml)
runs `npm run docs` in its Lyrical / Node 24 lane for pull requests and pushes.
The check builds the current checkout's API docs and landing page using the
publication template; invalid JSDoc annotations or generation errors fail the job.
It does not rebuild historical tags, upload artifacts or deploy to Pages.
The full publication dry run remains necessary to verify all tagged versions.

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
  published; the selected documentation/release tags are the content source.
