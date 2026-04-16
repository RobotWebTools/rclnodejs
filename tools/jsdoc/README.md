# JSDoc Workflow

This directory contains the custom JSDoc template, the landing-page generator,
and the staging script used to prepare the docs content that is published to the
`gh-pages` branch.

## Commands

### `npm run docs`

Build local docs for the current workspace version.

Output:

- `docs/<current-version>/`
- `docs/index.html`

Use this to verify the docs for the version currently declared in
`package.json`.

### `npm run docs:gh-pages`

Stage the publishable docs tree under `build/gh-pages-docs/`.

Behavior:

- reads the currently published version set from `origin/gh-pages`
- preserves that published history
- regenerates docs for the current workspace version
- rebuilds the staged landing page index

This is the normal command to use for a new release.

If you delete `build/` and rerun `npm run docs:gh-pages`, the staged tree will
still contain all currently published versions. That command recreates
`build/gh-pages-docs/` by copying the published docs snapshot from
`origin/gh-pages`, then regenerating only the current workspace version.

### `npm run docs:gh-pages:full`

Fully rebuild the currently published docs history under
`build/gh-pages-docs/`.

Behavior:

- reads the published version set from `origin/gh-pages`
- rebuilds only those published versions from tags
- regenerates docs for the current workspace version
- rebuilds the staged landing page index

This does **not** rebuild docs for every historical `rclnodejs` tag. It only
rebuilds the subset that is actually published online.

## New Release Example

For a new release such as `1.9.0`:

1. Update `package.json` to `1.9.0`.
2. Run `npm run docs`.
3. Verify the local output in `docs/1.9.0/` and `docs/index.html`.
4. Run `npm run docs:gh-pages`.
5. Verify the staged output in:
   - `build/gh-pages-docs/docs/1.9.0/`
   - `build/gh-pages-docs/docs/index.html`
   - `build/gh-pages-docs/.nojekyll`
6. Publish the contents of `build/gh-pages-docs/` to the `gh-pages` branch.

## GitHub Actions Deployment

The `deploy-docs.yml` workflow (`.github/workflows/deploy-docs.yml`) automates
building and deploying docs to GitHub Pages.

### Triggers

- **Tag push** matching `docs-*` (e.g. `git tag docs-1.9.0 && git push origin docs-1.9.0`) — builds and deploys automatically.
- **Manual dispatch** from the Actions tab — includes a `dry_run` toggle
  (defaults to `true`). Set it to `false` to deploy.

### What it does

1. Full checkout with all tags and the `origin/gh-pages` branch.
2. Runs `npm run docs:gh-pages` to stage the docs tree.
3. Uploads the staged output as a Pages artifact.
4. Deploys to GitHub Pages (skipped when `dry_run` is `true`).

### Testing

- Run the workflow manually with `dry_run` enabled to verify the build
  succeeds without deploying.
- Push a `docs-test` tag to trigger a full build + deploy, then clean up:
  `git tag -d docs-test && git push origin :refs/tags/docs-test`.

## Manual Landing Page Rebuild

If the staged docs tree already exists and you only want to rebuild
`build/gh-pages-docs/docs/index.html`, run `tools/jsdoc/build-index.js` against
that docs root and point it at the package metadata for the latest published
version.

Example for published version `1.8.0`:

```bash
mkdir -p build/gh-pages-docs/.tmp
git show 1.8.0:package.json > build/gh-pages-docs/.tmp/package-1.8.0.json

export RCLNODEJS_DOCS_ROOT="$PWD/build/gh-pages-docs/docs"
export RCLNODEJS_DOCS_INDEX_PATH="$PWD/build/gh-pages-docs/docs/index.html"
export RCLNODEJS_LOCAL_INDEX_PATH=''
export RCLNODEJS_PACKAGE_JSON_PATH="$PWD/build/gh-pages-docs/.tmp/package-1.8.0.json"

node tools/jsdoc/build-index.js
rm -rf build/gh-pages-docs/.tmp
```

## Notes

- The staged publish output keeps shared assets in `build/gh-pages-docs/docs/_static/`.
- `.nojekyll` must remain in the staged output because the published docs tree
  uses an underscore-prefixed directory.
- The live docs index is the source of truth for which versions should remain
  published.
