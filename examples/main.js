var scale = 4
  , svg
  , canvas
  , world
  , simulator;

function init(file) {
  create_html_elements();
  loadSVG(file);
}

function loadSVG(path) {
  var scene = s2p.SVGSceneParser.parseFile(path);

  // display svg scene
  svg.selectAll("*").remove();
  scene.renderInSvg(document, svg.node(), 0, 0, scale);

  // create & populate physics scene
  world = new Box2D.Dynamics.b2World(new Box2D.Common.Math.b2Vec2(0, 10), true);
  var adapter = new s2p.Box2DAdapter();
  scene.pixels_per_unit = 50;
  scene.friction = 0.3;
  scene.resitution = 0.1;
  adapter.loadScene(world, scene, true, false);

  // display physics scene
  simulator = new s2p.Simulator( new s2p.PhysicsScene(world), canvas.node()
                               , scene.pixels_per_unit*scale, false);
  simulator.play();
}

function create_html_elements() {
  svg = d3.select("body")
    .append("svg")
    .attr("width", 100*scale)
    .attr("height", 100*scale)
    .style("margin", "5px");

  canvas = d3.select("body")
    .append("canvas")
    .attr("width", 100*scale)
    .attr("height", 100*scale)
    .style("margin", "5px");
}
