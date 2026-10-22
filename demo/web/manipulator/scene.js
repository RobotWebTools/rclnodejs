// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
// Licensed under the Apache License, Version 2.0.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export function createManipulator() {
  const root = new THREE.Group();
  const mesh = (geometry, color) => {
    const object = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color,
        roughness: 0.35,
        metalness: 0.45,
      })
    );
    object.castShadow = true;
    object.receiveShadow = true;
    return object;
  };
  const base = mesh(new THREE.CylinderGeometry(0.8, 1, 0.5, 48), 0x617581);
  base.position.y = 0.25;
  root.add(base);
  for (let index = 0; index < 6; index++) {
    const angle = (index * Math.PI) / 3;
    const bolt = mesh(
      new THREE.CylinderGeometry(0.065, 0.065, 0.06, 6),
      0x283c45
    );
    bolt.position.set(Math.cos(angle) * 0.73, 0.49, Math.sin(angle) * 0.73);
    root.add(bolt);
  }

  const joint1 = new THREE.Group();
  joint1.position.y = 0.5;
  const shoulder = mesh(
    new THREE.CylinderGeometry(0.32, 0.37, 0.32, 32),
    0xdee6e5
  );
  shoulder.position.y = 0.14;
  joint1.add(shoulder);
  const link1 = mesh(new THREE.CylinderGeometry(0.18, 0.22, 2.5, 32), 0x6e99ad);
  link1.position.y = 1.25;
  joint1.add(link1);
  const inset = mesh(new THREE.BoxGeometry(0.16, 1.8, 0.035), 0xdce6e5);
  inset.position.set(0, 1.35, 0.2);
  joint1.add(inset);
  root.add(joint1);

  const joint2 = new THREE.Group();
  joint2.position.y = 2.5;
  const elbow = mesh(new THREE.CylinderGeometry(0.3, 0.3, 0.45, 32), 0xdce6e5);
  elbow.rotation.x = Math.PI / 2;
  joint2.add(elbow);
  for (const side of [-1, 1]) {
    const cap = mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.04, 32), 0x287a6b);
    cap.rotation.x = Math.PI / 2;
    cap.position.z = side * 0.24;
    joint2.add(cap);
  }
  const link2 = mesh(new THREE.CylinderGeometry(0.13, 0.16, 2, 32), 0xdce6e5);
  link2.position.y = 1;
  joint2.add(link2);
  joint1.add(joint2);

  const endEffector = mesh(new THREE.BoxGeometry(0.38, 0.25, 0.32), 0x304b55);
  endEffector.position.y = 2;
  for (const side of [-1, 1]) {
    const finger = mesh(new THREE.BoxGeometry(0.075, 0.34, 0.22), 0x71838d);
    finger.position.set(side * 0.16, 0.22, 0);
    endEffector.add(finger);
  }
  joint2.add(endEffector);

  const ring = (radius, color) =>
    new THREE.Mesh(
      new THREE.TorusGeometry(radius, 0.035, 8, 64),
      new THREE.MeshBasicMaterial({ color })
    );
  const baseRing = ring(1.2, 0x8ba39b);
  baseRing.rotation.x = Math.PI / 2;
  baseRing.position.y = 0.03;
  root.add(baseRing);
  const yawRing = ring(0.5, 0xd87868);
  yawRing.rotation.x = Math.PI / 2;
  joint1.add(yawRing);
  joint2.add(ring(0.36, 0x278c76));

  const heading = mesh(new THREE.ConeGeometry(0.09, 0.25, 12), 0xd87868);
  heading.rotation.z = -Math.PI / 2;
  heading.position.x = 0.7;
  joint1.add(heading);
  return { root, joint1, joint2, endEffector };
}

export function mountScene(container, toolContainer) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xe7eeeb);
  scene.fog = new THREE.Fog(0xe7eeeb, 18, 42);
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
  const toolCamera = new THREE.PerspectiveCamera(65, 1, 0.05, 100);
  const views = [
    { container, camera, label: 'Third-person manipulator view' },
    {
      container: toolContainer,
      camera: toolCamera,
      label: 'Simulated tool-mounted manipulator view',
    },
  ];
  for (const view of views) {
    view.renderer = new THREE.WebGLRenderer({ antialias: true });
    view.renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
    view.renderer.shadowMap.enabled = true;
    view.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    view.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    view.container.appendChild(view.renderer.domElement);
    view.renderer.domElement.setAttribute('aria-label', view.label);
  }

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8c9b91, 2));
  const light = new THREE.DirectionalLight(0xfff4e3, 3);
  light.position.set(5, 10, 5);
  light.castShadow = true;
  light.shadow.mapSize.set(1024, 1024);
  light.shadow.normalBias = 0.025;
  light.shadow.camera.left = -6;
  light.shadow.camera.right = 6;
  light.shadow.camera.top = 7;
  light.shadow.camera.bottom = -6;
  scene.add(light);
  const tile = new THREE.DataTexture(
    new Uint8Array([
      210, 220, 214, 255, 231, 237, 232, 255, 231, 237, 232, 255, 210, 220, 214,
      255,
    ]),
    2,
    2
  );
  tile.wrapS = tile.wrapT = THREE.RepeatWrapping;
  tile.repeat.set(16, 16);
  tile.magFilter = THREE.NearestFilter;
  tile.colorSpace = THREE.SRGBColorSpace;
  tile.needsUpdate = true;
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(40, 40),
    new THREE.MeshStandardMaterial({ map: tile, roughness: 0.9 })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.02;
  floor.receiveShadow = true;
  scene.add(floor);
  const axes = new THREE.AxesHelper(0.8);
  axes.position.set(-2, 0.04, 2);
  scene.add(axes);
  const arm = createManipulator();
  scene.add(arm.root);
  const controls = new OrbitControls(camera, views[0].renderer.domElement);
  controls.enableDamping = true;
  controls.minDistance = 4;
  controls.maxDistance = 18;
  controls.maxPolarAngle = Math.PI / 2 - 0.02;

  function resetCamera() {
    camera.position.set(6.5, 4.5, 7);
    controls.target.set(0, 2.3, 0);
    controls.update();
  }
  resetCamera();

  const resize = () => {
    for (const view of views) {
      const width = Math.max(view.container.clientWidth, 1);
      const height = Math.max(view.container.clientHeight, 1);
      view.camera.aspect = width / height;
      view.camera.updateProjectionMatrix();
      view.renderer.setSize(width, height, false);
    }
  };
  const observer = new ResizeObserver(resize);
  for (const view of views) observer.observe(view.container);
  resize();
  let animation;
  let frames = 0;
  let fps = 0;
  let lastFrameCheck = performance.now();
  const toolEye = new THREE.Vector3();
  const toolTarget = new THREE.Vector3();
  const toolRotation = new THREE.Quaternion();
  function frame() {
    controls.update();
    arm.root.updateMatrixWorld(true);
    arm.endEffector.localToWorld(toolEye.set(0, 0.4, 0.65));
    arm.joint2.localToWorld(toolTarget.set(0, -1, 0));
    arm.joint2.getWorldQuaternion(toolRotation);
    toolCamera.position.copy(toolEye);
    toolCamera.up.set(0, 0, -1).applyQuaternion(toolRotation);
    toolCamera.lookAt(toolTarget);
    for (const view of views) view.renderer.render(scene, view.camera);
    frames++;
    const now = performance.now();
    if (now - lastFrameCheck >= 1000) {
      fps = (frames * 1000) / (now - lastFrameCheck);
      frames = 0;
      lastFrameCheck = now;
    }
    animation = requestAnimationFrame(frame);
  }
  frame();

  return {
    setPositions([base, elbow]) {
      arm.joint1.rotation.y = base;
      arm.joint2.rotation.z = elbow;
    },
    endPosition() {
      arm.root.updateMatrixWorld(true);
      return arm.endEffector.getWorldPosition(new THREE.Vector3());
    },
    resetCamera,
    setNavigation(mode) {
      if (!['orbit', 'pan'].includes(mode))
        throw new TypeError('Unknown camera mode');
      controls.mouseButtons.LEFT =
        mode === 'orbit' ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN;
    },
    setDistance(distance) {
      if (!Number.isFinite(distance) || distance < 4 || distance > 18) {
        throw new RangeError('Camera distance must be between 4 and 18');
      }
      camera.position
        .sub(controls.target)
        .setLength(distance)
        .add(controls.target);
      controls.update();
    },
    stats() {
      return {
        fps,
        distance: camera.position.distanceTo(controls.target),
        width: views[0].renderer.domElement.width,
        height: views[0].renderer.domElement.height,
      };
    },
    dispose() {
      cancelAnimationFrame(animation);
      observer.disconnect();
      controls.dispose();
      const geometries = new Set();
      const materials = new Set();
      scene.traverse((object) => {
        if (object.geometry) geometries.add(object.geometry);
        if (object.material) {
          for (const material of Array.isArray(object.material)
            ? object.material
            : [object.material]) {
            materials.add(material);
          }
        }
      });
      for (const geometry of geometries) geometry.dispose();
      for (const material of materials) material.dispose();
      tile.dispose();
      for (const view of views) {
        view.renderer.dispose();
        view.renderer.domElement.remove();
      }
    },
  };
}
