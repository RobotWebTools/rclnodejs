// Copyright (c) 2026 The Robot Web Tools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const childProcess = require('child_process');
const fs = require('fs');
const path = require('path');

const repoRoot = path.resolve(__dirname, '../..');
const docsRoot = path.resolve(
  process.env.RCLNODEJS_DOCS_ROOT || path.join(repoRoot, 'docs')
);
const packageJsonPath = path.resolve(
  process.env.RCLNODEJS_PACKAGE_JSON_PATH || path.join(repoRoot, 'package.json')
);
const localIndexPath = process.env.RCLNODEJS_LOCAL_INDEX_PATH
  ? path.resolve(process.env.RCLNODEJS_LOCAL_INDEX_PATH)
  : null;
const docsIndexPath = path.resolve(
  process.env.RCLNODEJS_DOCS_INDEX_PATH || path.join(docsRoot, 'index.html')
);
const gitCwd = path.resolve(process.env.RCLNODEJS_GIT_CWD || repoRoot);

function readPackageInfo() {
  return JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
}

function isVersionDirectory(entryName) {
  return /^\d+\.\d+\.\d+$/.test(entryName);
}

function compareVersionsDesc(left, right) {
  const leftParts = left.split('.').map(Number);
  const rightParts = right.split('.').map(Number);
  const maxLength = Math.max(leftParts.length, rightParts.length);

  for (let index = 0; index < maxLength; index += 1) {
    const leftValue = leftParts[index] || 0;
    const rightValue = rightParts[index] || 0;

    if (leftValue !== rightValue) {
      return rightValue - leftValue;
    }
  }

  return 0;
}

function getAvailableVersions() {
  return fs
    .readdirSync(docsRoot, { withFileTypes: true })
    .filter((entry) => entry.isDirectory() && isVersionDirectory(entry.name))
    .map((entry) => entry.name)
    .sort(compareVersionsDesc);
}

function runGitCommand(args) {
  try {
    return childProcess
      .execFileSync('git', args, {
        cwd: gitCwd,
        encoding: 'utf8',
        stdio: ['ignore', 'pipe', 'ignore'],
      })
      .trim();
  } catch {
    return '';
  }
}

function getGitTags() {
  const tagsOutput = runGitCommand(['tag', '--list']);

  return new Set(tagsOutput ? tagsOutput.split('\n').filter(Boolean) : []);
}

function getTagReleaseDate(tagName) {
  const commitSha = runGitCommand(['rev-list', '-n', '1', tagName]);

  if (!commitSha) {
    return null;
  }

  return (
    runGitCommand(['show', '-s', '--format=%ad', '--date=short', commitSha]) ||
    null
  );
}

function buildVersionEntries(packageInfo, versions) {
  const gitTags = getGitTags();

  return versions.map((version) => {
    const hasTag = gitTags.has(version);

    return {
      version,
      releasedOn: hasTag ? getTagReleaseDate(version) : null,
      docsUrl: `./${version}/index.html`,
    };
  });
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function renderVersionCards(versionEntries, latestVersion) {
  return versionEntries
    .map((entry) => {
      const latestBadge =
        entry.version === latestVersion
          ? '<span class="version-badge">Latest</span>'
          : '';
      const releaseMeta = entry.releasedOn
        ? `<p class="version-meta">Released ${escapeHtml(entry.releasedOn)}</p>`
        : '';

      return `
        <a class="version-card" href="${escapeHtml(entry.docsUrl)}">
          <div class="version-card-header">
            <span class="version-label">Release</span>
            ${latestBadge}
          </div>
          <strong class="version-number">v${escapeHtml(entry.version)}</strong>
          ${releaseMeta}
          <p class="version-copy">Browse the generated API reference for rclnodejs ${escapeHtml(entry.version)}.</p>
        </a>`;
    })
    .join('');
}

function renderIndexHtml(packageInfo, versionEntries) {
  const latestEntry = versionEntries[0] || {
    version: packageInfo.version,
    releasedOn: null,
  };
  const latestVersion = latestEntry.version;
  const versionCards = renderVersionCards(versionEntries, latestVersion);
  const versionCount = versionEntries.length;
  const latestReleaseDate = latestEntry.releasedOn
    ? `<p>Released on <strong>${escapeHtml(latestEntry.releasedOn)}</strong></p>`
    : '<p>Release date unavailable for this version.</p>';

  return `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>rclnodejs Documentation</title>
  <style>
    :root {
      --bg: #f3f7fb;
      --bg-deep: #dfeaf4;
      --surface: rgba(255, 255, 255, 0.86);
      --surface-strong: rgba(255, 255, 255, 0.96);
      --border: rgba(28, 53, 77, 0.12);
      --border-strong: rgba(28, 53, 77, 0.18);
      --text: #12263a;
      --text-soft: #486277;
      --text-muted: #667f92;
      --accent: #0b84c9;
      --accent-strong: #08679b;
      --accent-soft: rgba(11, 132, 201, 0.12);
      --shadow-lg: 0 24px 60px rgba(14, 31, 53, 0.14);
      --shadow-md: 0 12px 30px rgba(14, 31, 53, 0.1);
      --radius-xl: 28px;
      --radius-lg: 20px;
      --radius-md: 14px;
    }

    * {
      box-sizing: border-box;
    }

    html {
      background: linear-gradient(180deg, #eff5fa 0%, #f7fbff 100%);
      color: var(--text);
      font-size: 16px;
    }

    body {
      margin: 0;
      font-family: "Open Sans", "Segoe UI", sans-serif;
      line-height: 1.6;
      color: var(--text);
      background: transparent;
    }

    a {
      color: inherit;
      text-decoration: none;
    }

    .site-backdrop {
      position: fixed;
      inset: 0;
      background:
        radial-gradient(circle at top left, rgba(11, 132, 201, 0.18), transparent 32%),
        radial-gradient(circle at right 10%, rgba(0, 128, 106, 0.12), transparent 24%),
        linear-gradient(180deg, rgba(255, 255, 255, 0.6), rgba(243, 247, 251, 0.95));
      pointer-events: none;
      z-index: -2;
    }

    .site-grid {
      position: fixed;
      inset: 0;
      background-image:
        linear-gradient(rgba(18, 38, 58, 0.03) 1px, transparent 1px),
        linear-gradient(90deg, rgba(18, 38, 58, 0.03) 1px, transparent 1px);
      background-size: 32px 32px;
      mask-image: linear-gradient(180deg, rgba(0, 0, 0, 0.28), transparent 75%);
      pointer-events: none;
      z-index: -1;
    }

    .page-shell {
      max-width: 1320px;
      margin: 0 auto;
      padding: 24px;
    }

    .hero,
    .panel,
    .footer {
      background: var(--surface-strong);
      border: 1px solid var(--border);
      border-radius: var(--radius-xl);
      box-shadow: var(--shadow-lg);
      backdrop-filter: blur(18px);
      -webkit-backdrop-filter: blur(18px);
    }

    .hero {
      display: grid;
      grid-template-columns: minmax(0, 1.35fr) minmax(280px, 0.8fr);
      gap: 24px;
      padding: 34px;
      overflow: hidden;
      position: relative;
      background:
        linear-gradient(135deg, rgba(255, 255, 255, 0.96), rgba(231, 243, 250, 0.9)),
        linear-gradient(120deg, rgba(11, 132, 201, 0.16), transparent 42%);
    }

    .hero::after {
      content: "";
      position: absolute;
      top: -50px;
      right: -50px;
      width: 200px;
      height: 200px;
      border-radius: 50%;
      background: radial-gradient(circle, rgba(11, 132, 201, 0.16), transparent 68%);
      pointer-events: none;
    }

    .eyebrow {
      margin: 0 0 10px;
      text-transform: uppercase;
      letter-spacing: 0.18em;
      font-size: 0.74rem;
      font-weight: 700;
      color: var(--accent-strong);
    }

    .hero h1 {
      margin: 0;
      font-size: clamp(2.8rem, 5vw, 4.8rem);
      line-height: 0.95;
      letter-spacing: -0.07em;
      font-weight: 300;
    }

    .hero h1 span {
      display: block;
      margin-top: 8px;
      color: var(--text-muted);
      font-size: 0.42em;
      font-weight: 700;
      letter-spacing: 0;
      text-transform: uppercase;
    }

    .hero-copy {
      position: relative;
      z-index: 1;
    }

    .hero-description,
    .panel p,
    .summary-card p,
    .version-copy,
    .version-meta,
    .footer {
      color: var(--text-soft);
    }

    .hero-description {
      max-width: 48rem;
      margin: 18px 0 0;
      font-size: 1.02rem;
    }

    .hero-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      margin-top: 22px;
    }

    .hero-action {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-height: 48px;
      padding: 0 16px;
      border-radius: 14px;
      border: 1px solid var(--border-strong);
      background: rgba(255, 255, 255, 0.72);
      color: var(--text);
      font-weight: 700;
      transition: transform 160ms ease, background-color 160ms ease, border-color 160ms ease;
    }

    .hero-action:hover {
      transform: translateY(-1px);
      background: rgba(255, 255, 255, 0.94);
      border-color: rgba(11, 132, 201, 0.22);
    }

    .hero-action.primary {
      background: linear-gradient(135deg, var(--accent), var(--accent-strong));
      border-color: transparent;
      color: #fff;
    }

    .panel {
      padding: 22px;
      position: relative;
      z-index: 1;
      background: rgba(248, 252, 255, 0.82);
      box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.8);
    }

    .panel-title {
      margin: 0 0 14px;
      color: var(--text);
      font-size: 0.92rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.12em;
    }

    .summary-grid,
    .versions-grid {
      display: grid;
      gap: 18px;
      margin-top: 24px;
    }

    .summary-grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }

    .summary-card,
    .version-card {
      background: var(--surface-strong);
      border: 1px solid var(--border);
      border-radius: var(--radius-lg);
      box-shadow: var(--shadow-md);
    }

    .summary-card {
      padding: 22px;
    }

    .summary-number {
      display: block;
      font-size: 2.2rem;
      line-height: 1;
      letter-spacing: -0.06em;
      font-weight: 300;
    }

    .summary-label {
      display: block;
      margin-top: 8px;
      font-weight: 700;
      color: var(--text);
    }

    .versions {
      margin-top: 24px;
      padding: 28px 30px;
    }

    .section-heading {
      margin: 0;
      font-size: clamp(1.8rem, 2.6vw, 2.6rem);
      line-height: 1.04;
      letter-spacing: -0.05em;
      font-weight: 300;
    }

    .section-copy {
      margin: 12px 0 0;
      max-width: 46rem;
      color: var(--text-soft);
    }

    .versions-grid {
      grid-template-columns: repeat(3, minmax(0, 1fr));
      margin-top: 22px;
    }

    .version-card {
      padding: 22px;
      transition: transform 160ms ease, border-color 160ms ease, box-shadow 160ms ease;
    }

    .version-card:hover {
      transform: translateY(-2px);
      border-color: rgba(11, 132, 201, 0.24);
      box-shadow: 0 18px 36px rgba(14, 31, 53, 0.12);
    }

    .version-card-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }

    .version-label,
    .version-badge {
      display: inline-flex;
      align-items: center;
      padding: 0.35rem 0.75rem;
      border-radius: 999px;
      font-size: 0.78rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.12em;
    }

    .version-label {
      background: var(--accent-soft);
      color: var(--accent-strong);
    }

    .version-badge {
      background: rgba(0, 128, 106, 0.12);
      color: #0d6a59;
    }

    .version-number {
      display: block;
      margin-top: 18px;
      font-size: 2rem;
      line-height: 1;
      letter-spacing: -0.06em;
      font-weight: 300;
      color: var(--text);
    }

    .version-meta {
      margin: 10px 0 0;
      font-size: 0.92rem;
      font-weight: 700;
      color: var(--text-muted);
    }

    .version-copy {
      margin: 12px 0 0;
    }

    .footer {
      margin-top: 24px;
      padding: 18px 22px;
      font-size: 0.92rem;
    }

    @media (max-width: 1100px) {
      .hero,
      .versions-grid,
      .summary-grid {
        grid-template-columns: 1fr 1fr;
      }

      .hero > :first-child {
        grid-column: 1 / -1;
      }
    }

    @media (max-width: 720px) {
      .page-shell {
        padding: 14px;
      }

      .hero,
      .versions-grid,
      .summary-grid {
        grid-template-columns: 1fr;
      }

      .hero,
      .versions,
      .panel,
      .footer,
      .summary-card,
      .version-card {
        border-radius: 22px;
      }

      .hero,
      .versions {
        padding: 22px 20px;
      }
    }
  </style>
</head>
<body>
  <div class="site-backdrop"></div>
  <div class="site-grid"></div>
  <div class="page-shell">
    <section class="hero">
      <div class="hero-copy">
        <p class="eyebrow">Versioned API Reference</p>
        <h1>rclnodejs <span>ROS 2 JavaScript Client</span></h1>
        <p class="hero-description">Browse published API references across released versions. The latest version is highlighted, and every docs build refreshes this landing page automatically.</p>
        <div class="hero-actions">
          <a class="hero-action primary" href="./${escapeHtml(latestVersion)}/index.html">Open Latest Docs</a>
          <a class="hero-action" href="#versions">Browse Versions</a>
        </div>
      </div>

      <aside class="panel">
        <p class="panel-title">Current Release</p>
        <h2 class="section-heading">v${escapeHtml(latestVersion)}</h2>
        <p>Package version from package.json: <strong>${escapeHtml(packageInfo.version)}</strong></p>
        ${latestReleaseDate}
      </aside>
    </section>

    <section class="summary-grid">
      <article class="summary-card">
        <strong class="summary-number">${versionCount}</strong>
        <span class="summary-label">Published Versions</span>
        <p>All discovered version directories under the docs output root.</p>
      </article>

      <article class="summary-card">
        <strong class="summary-number">v${escapeHtml(latestVersion)}</strong>
        <span class="summary-label">Latest Version</span>
        <p>The newest semantic version discovered during index generation, with release date information when available.</p>
      </article>
    </section>

    <section class="panel versions" id="versions">
      <p class="eyebrow">Releases</p>
      <h2 class="section-heading">Choose a documentation snapshot</h2>
      <p class="section-copy">Version entries are generated automatically from the directories currently present in the docs output, so new releases appear after <code>npm run docs</code> without manual HTML edits.</p>
      <div class="versions-grid">${versionCards}</div>
    </section>

    <footer class="footer">
      Documentation index for ${escapeHtml(packageInfo.name)}. Generated automatically by <code>tools/jsdoc/build-index.js</code>.
    </footer>
  </div>
</body>
</html>
`;
}

function writeIndexFiles(html) {
  if (localIndexPath) {
    fs.writeFileSync(localIndexPath, html, 'utf8');
  }
  fs.writeFileSync(docsIndexPath, html, 'utf8');
}

function main() {
  const packageInfo = readPackageInfo();
  const versions = getAvailableVersions();
  const versionEntries = buildVersionEntries(packageInfo, versions);
  const html = renderIndexHtml(packageInfo, versionEntries);

  writeIndexFiles(html);
}

main();
