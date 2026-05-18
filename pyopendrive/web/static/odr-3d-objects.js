export const odrObjectDefinitions = [
  { id: "traffic_light", label: "Traffic Light", icon: "fa-circle", signal_detail_kind: "post", object_color: "#344054" },
  { id: "stop_sign", label: "Stop Sign", icon: "fa-stop", name: "Stop_Sign", type: "stop_sign", length: 0.5, width: 0.5, height: 2.4, zOffset: 0, object_color: "#d92d20" },
  { id: "speed_sign", label: "Speed Sign", icon: "fa-tachometer", name: "Speed_Limit_Sign", type: "speed_sign", length: 0.5, width: 0.5, height: 2.4, zOffset: 0, object_color: "#fdb022" },
  { id: "street_light", label: "Street Light", icon: "fa-lightbulb-o", name: "Street_Light", type: "street_light", length: 0.5, width: 0.5, height: 7.5, zOffset: 0, object_color: "#667085" },
  { id: "bus_stop", label: "Bus Stop", icon: "fa-bus", name: "Bus_Stop", type: "bus_stop", length: 5.0, width: 2.2, height: 2.8, zOffset: 0, object_color: "#2e90fa" },
  { id: "crosswalk", label: "Crosswalk", icon: "fa-road", name: "Crosswalk", type: "crosswalk", length: 7.0, width: 3.0, height: 0.08, zOffset: 0, object_color: "#f2f4f7" },
  { id: "building", label: "Building", icon: "fa-building", name: "Building", type: "building", length: 12.0, width: 8.0, height: 8.0, zOffset: 0, object_color: "#8a6f4d" },
  { id: "apartment", label: "Apartment", icon: "fa-home", name: "Apartment", type: "apartment", length: 18.0, width: 12.0, height: 18.0, zOffset: 0, object_color: "#667085" },
  { id: "tree", label: "Tree", icon: "fa-tree", name: "Tree", type: "tree", length: 7.0, width: 7.0, height: 14.0, zOffset: 0, visual_scale: 2.4, object_color: "#16784f" },
];

function objectColor(props, fallback) {
  return props.object_color || fallback;
}

function material(THREE, color, options = {}) {
  return new THREE.MeshStandardMaterial({
    color,
    roughness: 0.82,
    metalness: 0.05,
    side: THREE.DoubleSide,
    ...options,
  });
}

function addBox(THREE, group, width, depth, height, color, x = 0, y = 0, z = height / 2) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, depth, height), material(THREE, color));
  mesh.position.set(x, y, z);
  group.add(mesh);
  return mesh;
}

function addCylinder(THREE, group, radius, height, color, x = 0, y = 0, z = height / 2, segments = 16) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, height, segments), material(THREE, color));
  mesh.rotation.x = Math.PI / 2;
  mesh.position.set(x, y, z);
  group.add(mesh);
  return mesh;
}

function addCone(THREE, group, radius, height, color, x = 0, y = 0, z = height / 2, segments = 24) {
  const mesh = new THREE.Mesh(new THREE.ConeGeometry(radius, height, segments), material(THREE, color));
  mesh.rotation.x = Math.PI / 2;
  mesh.position.set(x, y, z);
  group.add(mesh);
  return mesh;
}

function addTrafficLightModel(THREE, group, props) {
  const height = Math.max(3.2, Number(props.height) || 4.2);
  addCylinder(THREE, group, 0.06, height, "#475467");
  addBox(THREE, group, 0.42, 0.18, 0.9, "#111827", 0, 0, height + 0.1);
  for (const [index, color] of ["#d92d20", "#fdb022", objectColor(props, "#12b76a")].entries()) {
    const light = new THREE.Mesh(
      new THREE.SphereGeometry(0.095, 16, 12),
      material(THREE, color, { emissive: color, emissiveIntensity: 0.35 })
    );
    light.position.set(0, -0.1, height + 0.36 - index * 0.28);
    group.add(light);
  }
}

function addSignModel(THREE, group, props, fallbackColor) {
  const height = Math.max(2.0, Number(props.height) || 2.4);
  addCylinder(THREE, group, 0.045, height, "#667085");
  const sign = new THREE.Mesh(
    new THREE.CylinderGeometry(0.42, 0.42, 0.08, 8),
    material(THREE, objectColor(props, fallbackColor))
  );
  sign.rotation.x = Math.PI / 2;
  sign.position.set(0, -0.06, height + 0.15);
  group.add(sign);
}

function addStreetLightModel(THREE, group, props) {
  const height = Math.max(5.0, Number(props.height) || 7.5);
  const color = objectColor(props, "#667085");
  addCylinder(THREE, group, 0.055, height, color);
  addBox(THREE, group, 1.2, 0.08, 0.08, color, 0.55, 0, height);
  const lamp = new THREE.Mesh(
    new THREE.SphereGeometry(0.18, 16, 12),
    material(THREE, "#fdb022", { emissive: "#fdb022", emissiveIntensity: 0.4 })
  );
  lamp.position.set(1.15, 0, height - 0.08);
  group.add(lamp);
}

function addBusStopModel(THREE, group, props) {
  const length = Math.max(3.0, Number(props.length) || 5.0);
  const width = Math.max(1.4, Number(props.width) || 2.2);
  const height = Math.max(2.2, Number(props.height) || 2.8);
  const color = objectColor(props, "#2e90fa");
  addBox(THREE, group, length, 0.08, height, "#98a2b3", 0, -width / 2, height / 2);
  addBox(THREE, group, length, width, 0.12, color, 0, 0, height + 0.08);
  addBox(THREE, group, length * 0.72, 0.14, 0.45, "#344054", 0, width * 0.22, 0.45);
}

function addCrosswalkModel(THREE, group, props) {
  const length = Math.max(4.0, Number(props.length) || 7.0);
  const width = Math.max(2.0, Number(props.width) || 3.0);
  addBox(THREE, group, length, width, 0.03, objectColor(props, "#f2f4f7"), 0, 0, 0.02);
  for (let i = -2; i <= 2; i += 1) {
    addBox(THREE, group, length * 0.12, width, 0.04, "#111827", i * length * 0.18, 0, 0.06);
  }
}

function addBuildingModel(THREE, group, props, apartment = false) {
  const length = Math.max(4.0, Number(props.length) || (apartment ? 18 : 12));
  const width = Math.max(4.0, Number(props.width) || (apartment ? 12 : 8));
  const height = Math.max(4.0, Number(props.height) || (apartment ? 18 : 8));
  const color = objectColor(props, apartment ? "#667085" : "#8a6f4d");
  addBox(THREE, group, length, width, height, color);
  addBox(THREE, group, length * 1.06, width * 1.06, 0.28, "#344054", 0, 0, height + 0.14);
  const floors = Math.max(2, Math.floor(height / 3));
  const columns = Math.max(2, Math.floor(length / 3));
  for (let floor = 0; floor < floors; floor += 1) {
    for (let col = 0; col < columns; col += 1) {
      addBox(
        THREE,
        group,
        0.5,
        0.035,
        0.5,
        "#d0e8ff",
        -length / 2 + 1 + col * ((length - 2) / Math.max(1, columns - 1)),
        -width / 2 - 0.02,
        1.5 + floor * ((height - 2) / Math.max(1, floors - 1))
      );
    }
  }
}

function addTreeModel(THREE, group, props) {
  const height = Math.max(12.0, Number(props.height) || 14.0);
  const width = Math.max(5.5, Number(props.width) || 7.0);
  const length = Math.max(5.5, Number(props.length) || 7.0);
  const crownRadius = Math.max(2.1, Math.max(width, length) * 0.5);
  const trunkHeight = height * 0.72;
  const leafColor = objectColor(props, "#16784f");
  addCone(THREE, group, 0.45, trunkHeight, "#7a4f2a", 0, 0, trunkHeight * 0.5, 22);
  for (const [centerZ, layerHeight, radiusScale, color] of [
    [0.56, 0.46, 0.72, leafColor],
    [0.70, 0.42, 0.60, "#176b43"],
    [0.82, 0.36, 0.48, "#2b8f5e"],
    [0.93, 0.28, 0.34, "#3aa870"],
  ]) {
    const cone = addCone(THREE, group, crownRadius * radiusScale, height * layerHeight, color, 0, 0, height * centerZ, 32);
    cone.scale.y = 0.82;
  }
}

function addGenericObjectModel(THREE, group, props) {
  addBox(
    THREE,
    group,
    Math.max(0.5, Number(props.length) || 1),
    Math.max(0.5, Number(props.width) || 1),
    Math.max(0.5, Number(props.height) || 1),
    objectColor(props, "#8a6f4d")
  );
}

export function addOdrObjectShape(THREE, group, props) {
  const type = String(props.type || props.environment_type || props.name || "").toLowerCase();
  if (type.includes("traffic")) addTrafficLightModel(THREE, group, props);
  else if (type.includes("stop")) addSignModel(THREE, group, props, "#d92d20");
  else if (type.includes("speed")) addSignModel(THREE, group, props, "#fdb022");
  else if (type.includes("street_light")) addStreetLightModel(THREE, group, props);
  else if (type.includes("bus_stop")) addBusStopModel(THREE, group, props);
  else if (type.includes("crosswalk")) addCrosswalkModel(THREE, group, props);
  else if (type.includes("apartment")) addBuildingModel(THREE, group, props, true);
  else if (type.includes("building")) addBuildingModel(THREE, group, props, false);
  else if (type.includes("tree")) addTreeModel(THREE, group, props);
  else addGenericObjectModel(THREE, group, props);
}
