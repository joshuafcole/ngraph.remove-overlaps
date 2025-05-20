function removeOverlaps(layout, options) {
  if (!layout) throw new Error("layout argument is required");

  // This variable is used during iteration over quadTree
  var currentNode = {
    width: 0,
    height: 0,
    depth: 0,
    x: 0,
    y: 0,
    z: 0,
    body: null,
  };

  options = options || {};

  var defaultSize = options.defaultSize || {
    width: 10,
    height: 10,
    depth: 10,
  };

  var active = options.active !== undefined ? options.active : false;

  // we store amount of node movement into this variable.
  var totalMovement = 1;

  var physicsSimulator = layout.simulator;
  var graph = layout.graph;

  if (active) {
    layout.on("step", step);
    layout.on("disposed", dispose);
    layout.on("stable", removeAll);
  } else {
    removeAll();
  }

  // We will autodispose when layout is disposed. Just in case, giving a
  // reference for disposal
  return dispose;

  function dispose() {
    layout.off("step", step);
    layout.off("stable", removeAll);
    layout.off("disposed", dispose);
  }

  function removeAll() {
    while (totalMovement > 0) {
      runRemoveOverlapsIfNeeded();
    }
  }

  function step() {
    runRemoveOverlapsIfNeeded();
  }

  function runRemoveOverlapsIfNeeded() {
    var tree = physicsSimulator.quadTree;
    var root = tree.getRoot();
    totalMovement = 0;

    layout.forEachBody(function (body, graphNodeId) {
      var graphNode = graph.getNode(graphNodeId);

      currentNode.width = getWidth(graphNode.data);
      currentNode.height = getHeight(graphNode.data);
      currentNode.depth = getDepth(graphNode.data);
      currentNode.x = body.pos.x;
      currentNode.y = body.pos.y;
      currentNode.z = body.pos.z;
      currentNode.body = body;

      traverse(root, visitQuad);
    });
  }

  function visitQuad(quad) {
    var body = quad.body;
    if (body === currentNode.body) return false;

    if (body) {
      // Check if body needs to be moved;
      totalMovement += moveIfNeeded(body);
    } else {
      // we only continue subdividing the tree if current node intersects the quad
      return intersectQuad(quad);
    }
  }

  function getWidth(nodeData) {
    return nodeData?.width ?? defaultSize.width;
  }

  function getHeight(nodeData) {
    return nodeData?.height ?? defaultSize.height;
  }

  function getDepth(nodeData) {
    return nodeData?.depth ?? defaultSize.depth;
  }

  function moveIfNeeded(body) {
    var a = currentNode;
    var node = graph.getNode(body.id);

    var b = {
      x: body.pos.x,
      y: body.pos.y,
      z: body.pos.z,
      width: getWidth(node?.data),
      height: getHeight(node?.data),
      depth: getDepth(node?.data),
    };

    var ox =
      Math.min(a.x + a.width / 2, b.x + b.width / 2) -
      Math.max(a.x - a.width / 2, b.x - b.width / 2);
    var oy =
      Math.min(a.y + a.height / 2, b.y + b.height / 2) -
      Math.max(a.y - a.height / 2, b.y - b.height / 2);
    var oz =
      Math.min(a.z + a.depth / 2, b.z + b.depth / 2) -
      Math.max(a.z - a.depth / 2, b.z - b.depth / 2);

    if (ox > 0 && oy > 0 && oz > 0) {
      var totalMovement = move(ox, oy, oz, a, b);
      currentNode.body.pos.x = a.x;
      currentNode.body.pos.y = a.y;
      currentNode.body.pos.z = a.z;

      body.pos.x = b.x;
      body.pos.y = b.y;
      body.pos.z = b.z;

      physicsSimulator.quadTree.updateBodyForce(a.body);
      physicsSimulator.quadTree.updateBodyForce(body);

      return totalMovement;
    }

    return 0;
  }

  function move(ox, oy, oz, a, b) {
    // Check for 3D intersection, but only move in 2D (xy plane)
    // shift along the axis of ideal/target positions in xy plane
    // so boxes can cross each other rather than collide

    var vx0 = a.x + a.width / 2 - (b.x + b.width / 2);
    var vy0 = a.y + a.height / 2 - (b.y + b.height / 2);
    // var vz0 = a.z + a.depth / 2 - (b.z + b.depth / 2);

    // We use intersection area to determine magnitude
    var shift = Math.sqrt(ox * oy),
      shiftX,
      shiftY;

    // Project the direction vector onto the xy plane
    var v0 = Math.hypot(vx0, vy0);

    if (v0 !== 0) {
      // Normalize the xy components
      vx0 /= v0;
      vy0 /= v0;
    } else {
      // Random direction in 2D space (xy plane)
      var phi = Math.random() * 2 * Math.PI;
      vx0 = Math.cos(phi);
      vy0 = Math.sin(phi);
    }

    shiftX = shift * vx0;
    shiftY = shift * vy0;

    // Only move in xy plane
    a.x += shiftX;
    b.x -= shiftX;
    a.y += shiftY;
    b.y -= shiftY;

    return Math.abs(shiftX) + Math.abs(shiftY);
  }

  function move3d(ox, oy, oz, a, b) {
    // shift along the axis of ideal/target positions
    // so boxes can cross each other rather than collide
    // this makes the result more predictable

    var vx0 = a.x + a.width / 2 - (b.x + b.width / 2);
    var vy0 = a.y + a.height / 2 - (b.y + b.width / 2);
    var vz0 = a.z + a.depth / 2 - (b.z + b.depth / 2);
    var v0 = Math.hypot(vx0, vy0, vz0);
    var shift = Math.cbrt(ox * oy * oz);
    var shiftX, shiftY, shiftZ;

    if (v0 !== 0) {
      vx0 /= v0;
      vy0 /= v0;
      vz0 /= v0;
    } else {
      var theta = Math.random() * 2 * Math.PI; // azimuthal angle
      var phi = Math.random() * Math.PI; // polar angle
      vx0 = Math.sin(phi) * Math.cos(theta);
      vy0 = Math.sin(phi) * Math.sin(theta);
      vz0 = Math.cos(phi);
    }

    shiftX = shift * vx0;
    shiftY = shift * vy0;
    shiftZ = shift * vz0;

    a.x += shiftX;
    b.x -= shiftX;
    a.y += shiftY;
    b.y -= shiftY;
    a.z += shiftZ;
    b.z -= shiftZ;

    return Math.abs(shiftX) + Math.abs(shiftY) + Math.abs(shiftZ);
  }

  function intersectQuad(quad) {
    var xHalf = currentNode.width / 2;
    var yHalf = currentNode.height / 2;
    var left = currentNode.x - xHalf;
    var top = currentNode.y - yHalf;
    var right = currentNode.x + xHalf;
    var bottom = currentNode.y + yHalf;

    // Continue subdivision only if current rectangle intersects our quad
    // http://stackoverflow.com/questions/306316/determine-if-two-rectangles-overlap-each-other
    return (
      left < quad.max_x &&
      right > quad.min_x &&
      top < quad.max_y &&
      bottom > quad.min_y
    );
  }

  function traverse(quad, visitor) {
    if (visitor(quad)) {
      if (quad.quad0) traverse(quad.quad0, visitor);
      if (quad.quad1) traverse(quad.quad1, visitor);
      if (quad.quad2) traverse(quad.quad2, visitor);
      if (quad.quad3) traverse(quad.quad3, visitor);
    }
  }
}

export default removeOverlaps;
