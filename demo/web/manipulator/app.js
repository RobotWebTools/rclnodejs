// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
// Licensed under the Apache License, Version 2.0.

import { RosClient } from '/sdk/index.js';
import { LIMITS, readJointPositions } from './joint-state.js';
import { mountScene } from './scene.js';

const element = (id) => document.getElementById(id);
const status = element('connection-status');
const errorOutput = element('error');
const retry = element('reconnect');
let client;
let view;
let generation = 0;
let connected = false;
let lastMessage;
let messageCount = 0;
let rateCount = 0;
let rejectedCount = 0;
let lastRateCheck = performance.now();
let telemetryTimer;

function setStatus(text, state) {
  if (status.textContent !== text) status.textContent = text;
  if (status.dataset.state !== state) status.dataset.state = state;
}

function reportError(error) {
  console.error(error);
  errorOutput.textContent = error.message || String(error);
  errorOutput.hidden = false;
}

function receive(message) {
  let positions;
  try {
    positions = readJointPositions(message);
  } catch (error) {
    if (!(error instanceof TypeError)) throw error;
    element('rejected-count').textContent = String(++rejectedCount);
    if (errorOutput.hidden || errorOutput.textContent !== error.message)
      reportError(error);
    setStatus('Invalid joint state', 'error');
    return;
  }
  view.setPositions(positions);
  positions.forEach((value, index) => {
    element(`joint${index + 1}-value`).textContent =
      `${((value * 180) / Math.PI).toFixed(1)}\u00b0`;
    element(`joint${index + 1}-radians`).textContent =
      `${value.toFixed(3)} rad`;
    element(`joint${index + 1}-bar`).style.width =
      `${Math.max(0, Math.min(100, ((value + LIMITS[index]) / (2 * LIMITS[index])) * 100))}%`;
  });
  const end = view.endPosition();
  element('end-position').textContent =
    `${end.x.toFixed(2)}, ${end.y.toFixed(2)}, ${end.z.toFixed(2)}`;
  const history = element('recent-states');
  if (history.querySelector('.empty-state')) history.replaceChildren();
  const row = document.createElement('tr');
  for (const value of [
    new Date().toLocaleTimeString([], { hour12: false }),
    ...positions.map((position) => ((position * 180) / Math.PI).toFixed(1)),
    ...[end.x, end.y, end.z].map((coordinate) => coordinate.toFixed(2)),
  ]) {
    const cell = document.createElement('td');
    cell.textContent = value;
    row.appendChild(cell);
  }
  history.prepend(row);
  while (history.rows.length > 5) history.lastElementChild.remove();
  lastMessage = performance.now();
  element('message-count').textContent = String(++messageCount);
  rateCount++;
  errorOutput.hidden = true;
  setStatus('Live', 'live');
}

async function startConnection(url, topic) {
  const current = ++generation;
  retry.disabled = true;
  connected = false;
  setStatus('Connecting', 'waiting');
  if (client) await client.close();
  const next = new RosClient(url, { reconnect: true });
  client = next;
  const active = () => generation === current;
  next.on('disconnected', () => {
    if (!active()) return;
    connected = false;
    setStatus('Disconnected', 'error');
    retry.disabled = false;
  });
  next.on('reconnecting', ({ attempt }) => {
    if (active()) setStatus(`Reconnecting (${attempt})`, 'waiting');
  });
  next.on('reconnected', () => {
    if (!active()) return;
    connected = true;
    setStatus('Waiting for state', 'waiting');
    retry.disabled = false;
  });
  try {
    await next.connect();
    if (!active()) {
      await next.close();
      return;
    }
    await next.subscribe(topic, (message) => {
      if (active()) receive(message);
    });
    if (!active()) {
      await next.close();
      return;
    }
    connected = true;
    setStatus('Waiting for state', 'waiting');
  } catch (error) {
    if (active()) {
      setStatus('Connection failed', 'error');
      reportError(error);
    }
    await next.close();
  } finally {
    if (active()) retry.disabled = false;
  }
}

async function main() {
  view = mountScene(element('viewport'), element('tool-viewport'));
  element('reset-camera').addEventListener('click', view.resetCamera);
  for (const mode of ['orbit', 'pan']) {
    element(`${mode}-mode`).addEventListener('click', () => {
      view.setNavigation(mode);
      for (const candidate of ['orbit', 'pan']) {
        element(`${candidate}-mode`).setAttribute(
          'aria-pressed',
          String(candidate === mode)
        );
      }
    });
  }
  element('camera-distance').addEventListener('input', (event) => {
    view.setDistance(Number(event.target.value));
  });
  const response = await fetch('/config.json');
  if (!response.ok)
    throw new Error(
      `Unable to read runtime configuration (${response.status})`
    );
  const config = await response.json();
  const url = new URL('/capability', location.href);
  url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  url.port = String(config.runtimePort);
  element('endpoint').textContent = url.href;
  element('topic-name').textContent = config.topic;
  retry.addEventListener('click', () => {
    startConnection(url.href, config.topic).catch(reportError);
  });
  telemetryTimer = setInterval(() => {
    const now = performance.now();
    element('frequency').textContent =
      `${((rateCount * 1000) / (now - lastRateCheck)).toFixed(1)} Hz`;
    lastRateCheck = now;
    rateCount = 0;
    const rendering = view.stats();
    element('render-rate').textContent = `${rendering.fps.toFixed(0)} fps`;
    element('preview-size').textContent =
      `${rendering.width} \u00d7 ${rendering.height}`;
    element('camera-distance').value = String(rendering.distance);
    const age = lastMessage === undefined ? null : now - lastMessage;
    element('last-message').textContent =
      age === null ? 'No data yet' : `${(age / 1000).toFixed(1)} s ago`;
    if (connected && (age === null || age > 2000)) {
      setStatus('Waiting for state', 'waiting');
    }
  }, 1000);
  await startConnection(url.href, config.topic);
}

window.addEventListener('pagehide', () => {
  generation++;
  clearInterval(telemetryTimer);
  if (client) client.close().catch(reportError);
  view?.dispose();
});

main().catch((error) => {
  setStatus('Unable to start', 'error');
  reportError(error);
});
