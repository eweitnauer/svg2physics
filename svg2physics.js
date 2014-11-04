s2p = (function(){
  var s2p = { version: "1.1.1" }; // semver
// Copyright Erik Weitnauer 2014.
var
  b2BodyDef = Box2D.Dynamics.b2BodyDef
 ,b2Body = Box2D.Dynamics.b2Body
 ,b2Vec2 = Box2D.Common.Math.b2Vec2
 ,b2Shape = Box2D.Collision.Shapes.b2Shape
 ,b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
 ,b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
 ,b2FixtureDef = Box2D.Dynamics.b2FixtureDef
 ,b2AABB = Box2D.Collision.b2AABB;

Box2DAdapter = function() {
  var self = this;
  this.rel_curve_error_margin = 0.08;  // unit: fraction of object diameter
  this.linear_damping = 0.35;   // Set for each body. Box2D uses the formula
  this.angular_damping = 0.35;  // angularVelocity *= b2Math.Clamp(1.0 - dt * angularDamping, 0.0, 1.0)
                                // A value of 0.35 will therefore lead to halving of the speed every
                                // two seconds in case of an update rate of 60 Hz.
  this.cd_settings = {
    max_vertices: 32
  , preprocess: true
  , pre_order_vertices: true
  , pre_merge_vertices_min_dist: 0.01 // in units
  , pre_remove_vertices_max_error: 0 // in units
  , postprocess: false
  , post_move_vertices_inside_dist: 0.02
  , debug_text: false
  };
}

/** Iterates through the passed scene's shapes and creates b2Bodys from them. Polygon shapes
    must be centered at the origin and have x, y properties to denote their position. Each
    shape must have the movable property set to true, if it is dynamic object. The
    b2Bodys are added to the passed b2World and also written to the 'phys_obj' attribute of
    their respective shape. To the b2Bodys, an attribute 'master_obj' is set to the shape for
    which it the b2Body was created. The scale that was used to create the objects is written
    to them as phys_scale attribute. A method synch_to_phys to synch the x, y and rot attribute
    with their b2Body is added.
    Parameters:
      world                  b2World
      scene                  an SVGScene object with an Array of Polygons and Circles */
Box2DAdapter.prototype.loadScene = function(world, scene) {
  // get friction, restitution and scale from the properties read from the SVG
  var friction = scene.friction;
  var restitution = scene.restitution;
  var scale = 1/scene.pixels_per_unit;

  var self = this;
  var first = true;
  // // add the frame
  // var frame = new Polygon([[0,0],[100*scale, 0],[100*scale, 100*scale],[0, 100*scale]]);
  // frame.closed = true;
  // self.createBody(world, frame, false, 0, 0, 0, 1, friction, restitution);

  // now add all other shapes
  scene.shapes.forEach(function(shape) {
    var stroke_width = 1;
    var reg_float = /^[0-9]*\.?[0-9]+/;
    if (reg_float.test(shape.style['stroke-width'])) {
      stroke_width = Number(reg_float.exec(shape.style['stroke-width'])[0]);
    }
    var target_width = Box2D.Common.b2Settings.b2_linearSlop;
    var _shape = shape.copy();
    // We now scale the shape according to the scenes' pixel per unit scale factor
    // It is important that objects that are stacked onto each other are also stacked exacty only each
    // other in the physics simulation. Box2D has a threshold b2_linearSlop used for collision and
    // constraint resolution. The border width of the shapes should be equal to this value. We will grow /
    // shrink the movable shapes and their borders so this is fulfilled.
    var bb = shape.bounding_box();
    var scale_x = (bb.width + stroke_width) / (bb.width + target_width);
    var scale_y = (bb.height + stroke_width) / (bb.height + target_width);
    if (_shape instanceof Polygon) {
      _shape.pts.forEach(function(p) { p.Scale(scale) });
      if (shape.movable) {
        //console.log('scale', scale_x, scale_y);
        _shape.pts.forEach(function(p) { p.x *= scale_x; p.y *= scale_y });
      } else if (shape.id == "_") {
         // if its the ground, move it up a bit
         _shape.pts.forEach(function(p) { p.y += (target_width-stroke_width)*scale/4 });
      }
    } else if (_shape instanceof Circle) {
      // since the b2Circle_shape as no additional collision radius as the polygon,
      // we will grow the circle to make it the same size as its SVG source,
      // which means we need to include half of its stroke-width
      _shape.r = (_shape.r+stroke_width/2) * scale;
    } else throw("Unknown object type.");
    shape.phys_scale = scale;
    /// Method that sets x, y and rot attributes of the scene object according to
    /// the state of its b2Body.
    shape.synch_to_phys = function() {
      this.x = this.phys_obj.GetPosition().x / this.phys_scale;
      this.y = this.phys_obj.GetPosition().y / this.phys_scale;
      this.rot = this.phys_obj.GetAngle();
    }
    shape.phys_obj = self.createBody(world, _shape, shape.movable, shape.x*scale, shape.y*scale,
                                     0.0, 1.0, friction, restitution);
    shape.phys_obj.master_obj = shape;
    shape.rot = 0;
  });
}

/// Creates a Box2D body according to the passed geometric and dynamic
/// information. The body is added to the world and returned.
/** Params:
  *   world: b2World
  *   shape: Polygon
  *   is_dynamic: bool
  *   x, y, angle, density, friction, restitution: number
  */
Box2DAdapter.prototype.createBody = function(world, shape, is_dynamic, x, y,
    angle, density, friction, restitution)
{
  // create the body
  var bodyDef = new b2BodyDef();
  if (is_dynamic) bodyDef.type = b2Body.b2_dynamicBody;
  bodyDef.position.Set(x,y);
  bodyDef.angle = angle;
  bodyDef.angularDamping = this.angular_damping;
  bodyDef.linearDamping = this.linear_damping;
  var body = world.CreateBody(bodyDef);

  // shape information
  var fixture_proto = {density: density, friction: friction, restitution: restitution};
  this.add_fixture(shape, body, fixture_proto, is_dynamic);

  return body;
}

/// Creates Box2D shapes from the passed polygon shape and adds it to the body.
/** If the body is dynamic, the polygon is decomposed into convex parts with no
  * more than 'max_polygon_vertices' vertices. For each of these parts a
  * b2PolygonShape is put into a fixture based on the passed fixture prototype.
  * The fixtures are then assigned to the body.
  *
  * If the body is not dynamic, either a single b2LoopShape (in case the polygon
  * is closed) or several b2EdgeShapes (one for every edge in case the polygon
  * is not closed) are used to create the fixtures. This is not possible in the
  * dynamic case, since b2LoopShapes and b2EdgeShapes are not allowed to be
  * dynamic in Box2D.
  * Params:
  *   shape: Polygon or Circle
  *   body: b2Body (to which the shape is added as fixture)
  *   fixture_proto: object (all properties are copied into the new fixture)
  *   is_dynamic: bool
  */
Box2DAdapter.prototype.add_fixture = function(shape, body, fixture_proto, is_dynamic) {
  if (shape instanceof Circle) {
    // shape is a circle
    var b2shape = new b2CircleShape(shape.r);
    var to_add = new b2FixtureDef();
    obj_extend(to_add, fixture_proto);
    to_add.shape = b2shape;
    body.CreateFixture(to_add);
  } else if (shape instanceof Polygon) {
    var poly = shape;
    // it is a polygon, we need to create different Box2D shape types for
    // dynamic and static case
    if (is_dynamic) {
      // do convex decomposition and add a b2PolygonShape for each piece
      var convex_polys = poly.convex_decomposition(this.cd_settings);
      convex_polys.forEach(function(p) {
        var b2shape = b2PolygonShape.AsVector(p.pts);
        var to_add = new b2FixtureDef();
        obj_extend(to_add, fixture_proto);
        to_add.shape = b2shape;
        body.CreateFixture(to_add);
      });
    } else {
      // simplify the poly a bit
      //poly.merge_vertices({min_dist: this.cd_settings.pre_merge_vertices_min_dist})
      // there are no working versions of b2EdgeShape or b2LoopShape in Box2D.js
      // so we use Polygons with 2 points instead
      // body is going to be static
      var N = poly.pts.length;
      if (N<2) return;
      for(var i=0; i<N; ++i) {
        if (i==N-1 && !poly.closed) break;
        var j = (i==N-1) ? 0 : i+1;
        var b2shape = b2PolygonShape.AsVector([new b2Vec2(poly.pts[i].x, poly.pts[i].y),
                                             new b2Vec2(poly.pts[j].x, poly.pts[j].y)]);
        var to_add = new b2FixtureDef();
        obj_extend(to_add, fixture_proto);
        to_add.shape = b2shape;
        body.CreateFixture(to_add);
      }
    }
  } else throw("Unkown shape type!");
}

obj_extend = function(target, src) {
  for (x in src) {
    if (src.hasOwnProperty(x)) target[x] = src[x];
  }
}

obj_extended = function(src1, src2) {
  var target = {};
  for (x in src1) {
    if (src1.hasOwnProperty(x)) target[x] = src1[x];
  }
  for (x in src2) {
    if (src2.hasOwnProperty(x)) target[x] = src2[x];
  }
  return target;
}

s2p.Box2DAdapter = Box2DAdapter;// Copyright Erik Weitnauer 2014.
var b2Body = Box2D.Dynamics.b2Body
   ,b2World = Box2D.Dynamics.b2World
   ,b2Transform = Box2D.Common.Math.b2Transform
   ,b2Sweep = Box2D.Common.Math.b2Sweep
   ,b2DistanceInput = Box2D.Collision.b2DistanceInput
   ,b2DistanceOutput = Box2D.Collision.b2DistanceOutput
   ,b2DistanceProxy = Box2D.Collision.b2DistanceProxy
   ,b2SimplexCache = Box2D.Collision.b2SimplexCache
   ,b2Distance = Box2D.Collision.b2Distance
   ,b2Vec2 = Box2D.Common.Math.b2Vec2
   ,b2BodyDef = Box2D.Dynamics.b2BodyDef
   ,b2FixtureDef = Box2D.Dynamics.b2FixtureDef
   ,b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
   ,b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
   ,b2AABB = Box2D.Collision.b2AABB;

b2Vec2.prototype.Transformed = function(xf) {
  return new b2Vec2(this.x*xf.R.col1.x + this.y*xf.R.col2.x + xf.position.x,
                    this.x*xf.R.col1.y + this.y*xf.R.col2.y + xf.position.y);
}

b2Body.prototype.IsCircle = function() {
  return this.m_fixtureList.m_shape instanceof b2CircleShape && this.m_fixtureList.m_next == null;
}

/// Returns the minimal distance between this and the passed body.
b2Body.prototype.distance = function(other) {
  var dist_fn = function(shape1, transform1, shape2, transform2) {
    var input = new b2DistanceInput();
    input.proxyA = new b2DistanceProxy();
    input.proxyA.Set(shape1);
    input.proxyB = new b2DistanceProxy();
    input.proxyB.Set(shape2);
    input.transformA = transform1;
    input.transformB = transform2;
    input.useRadii = true;
    var simplexCache = new b2SimplexCache();
    simplexCache.count = 0;
    var output = new b2DistanceOutput();
    b2Distance.Distance(output, simplexCache, input);
    return output.distance;
  }
  var min_dist = Infinity;
  for (var fix1=this.m_fixtureList; fix1; fix1=fix1.m_next) {
    for (var fix2=other.m_fixtureList; fix2; fix2=fix2.m_next) {
      var dist = dist_fn(fix1.m_shape, this.GetTransform(), fix2.m_shape, other.GetTransform())
      if (min_dist > dist) min_dist = dist;
    }
  }
  return min_dist;
}

/// Will set the categoryBits, maskBits and groupIndex fields passed in
/// the filter_options object. If an field is not set, the old values of the
/// filter will be kept.
/// Example: setCollisionFilter({maskBits: 0x0000}).
/// See http://www.iforce2d.net/b2dtut/collision-filtering.
b2Body.prototype.setCollisionFilter = function(filter_options) {
  var filter;
  for (var f=this.m_fixtureList; f; f = f.m_next) {
    filter = f.GetFilterData();
    if ('maskBits' in filter_options) filter.maskBits = filter_options.maskBits;
    if ('categoryBits' in filter_options) filter.categoryBits = filter_options.categoryBits;
    if ('groupIndex' in filter_options) filter.groupIndex = filter_options.groupIndex;
    f.SetFilterData(filter);
  }
}

function b2BodyState(body) {
  this.Init(body);
}

b2BodyState.prototype.Init = function(body) {
  this.m_flags = body.m_flags;
  this.m_xf = new b2Transform(); this.m_xf.Set(body.m_xf);
  this.m_sweep = new b2Sweep(); this.m_sweep.Set(body.m_sweep);
  this.m_linearVelocity = body.m_linearVelocity.Copy();
  this.m_angularVelocity = body.m_angularVelocity;
	this.m_linearDamping = body.m_linearDamping;
  this.m_angularDamping = body.m_angularDamping;
  this.m_force = body.m_force.Copy();
  this.m_torque = body.m_torque;
	this.m_sleepTime = body.m_sleepTime;
	this.m_type = body.m_type;
	this.m_mass = body.m_mass;
	this.m_invMass = body.m_invMass;
	this.m_I = body.m_I;
	this.m_invI = body.m_invI;
	this.m_inertiaScale = body.m_inertiaScale;
	this.m_islandIndex = body.m_islandIndex;
}

b2BodyState.prototype.Apply = function(body) {
  body.m_xf.Set(this.m_xf);
  body.m_sweep.Set(this.m_sweep);
  body.m_linearVelocity = this.m_linearVelocity.Copy();
  body.m_angularVelocity = this.m_angularVelocity;
	body.m_linearDamping = this.m_linearDamping;
  body.m_angularDamping = this.m_angularDamping;
  body.m_force = this.m_force.Copy();
  body.m_torque = this.m_torque;
	body.m_sleepTime = this.m_sleepTime;
	body.m_type = this.m_type;
	body.m_mass = this.m_mass;
	body.m_invMass = this.m_invMass;
	body.m_I = this.m_I;
	body.m_invI = this.m_invI;
	body.m_inertiaScale = this.m_inertiaScale;
	body.m_islandIndex = this.m_islandIndex;
  if ((this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag) {
    body.SetActive(true);
  }
  if ((this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag) {
    body.SetAwake(true);
  }
  body.m_flags = this.m_flags;
  body.SynchronizeFixtures();
  this.moveAwayAndBackAgain(body); // needed to keep objects passing through
                                   // each other when toggling type between
                                   // static and dynamic
}

b2BodyState.prototype.moveAwayAndBackAgain = function(body) {
  body.SetPosition({x: Infinity, y: Infinity});
  body.SetPosition(this.m_xf.position);
}

function b2WorldState(world) {
  this.Init(world);
}

b2WorldState.prototype.Init = function(world) {
  // curr_time is no internal property, we have to keep track of it
  // ourselves each time we step the world
  this.curr_time = world.curr_time;
}

b2WorldState.prototype.Apply = function(world) {
  world.curr_time = this.curr_time;
}

b2Body.prototype.PushState = function() {
  if (!this.bodystates) this.bodystates = [];
  this.bodystates.push(new b2BodyState(this));
}

b2Body.prototype.PopState = function() {
  this.bodystates.pop().Apply(this);
}

/// Pushes the states of all dynamic bodies.
b2World.prototype.PushState = function() {
  if (!this.worldstates) this.worldstates = [];
  this.worldstates.push(new b2WorldState(this));
  for (var b = this.m_bodyList; b; b=b.m_next) b.PushState();
}

/// Pops the states of all dynamic bodies.
b2World.prototype.PopState = function() {
  this.worldstates.pop().Apply(this);
  for (var b = this.m_bodyList; b; b=b.m_next) b.PopState();
  this.m_contactManager.FindNewContacts();
}

/// Saves the state of the b2World and all dynamic b2Bodies into an array and returns it.
b2World.prototype.GetState = function() {
  var state = [];
  state.push({el: this, state: new b2WorldState(this)});
  for (var b = this.m_bodyList; b; b=b.m_next) {
    state.push({el: b, state: new b2BodyState(b)});
  }
  return state;
}

/// Pass the result of a former GetState call to set the world to that situation.
b2World.prototype.SetState = function(state) {
  state.forEach(function (e) { e.state.Apply(e.el) });
}
// Copyright Erik Weitnauer 2014.

var b2DebugDraw = Box2D.Dynamics.b2DebugDraw
  , b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;

/** Visualization of a PhysicsScene on a canvas with mouse interaction.
In the constructor, you can pass a scaling, whether the time
should be shown and whether the simulator should pause automatically when
during playing, all bodies in the scene become inactive.

Call play(), pause() and reset() to control the simulator. The drawing attribute
can be set to false to disable the drawing. If drawing is enabled, the simulator
will automatically redraw the scene if anything in it changed (e.g., it was stepped
from someone other than the Simulator).

Mouse interaction is also possible when paused, the scene will be stepped during the
interaction. If the pause was due to autopause, the simulator will switch back into
play mode when the user starts interacting.

Manually set show_pos to true in order to display the current mouse position.
*/
Simulator = function(physics_scene, canvas, scaling, show_time, auto_pause) {
  this.canvas = canvas;
  this.ctx = canvas.getContext('2d');
  this.pscene = physics_scene;
  this.step_interval = 1000/30;        // frequency of stepping the scene when playing in ms
  this.interaction_interval = 1000/30; // frequency of updating the mouse interaction in ms
  this.show_time = show_time || true;  // displays current time
  this.show_pos = false;               // displays mouse pos. in world coordinates
  this.draw_scale = scaling || 1;
  this.playing = false;
  this.drawing = true;
  this.auto_pause = (auto_pause === undefined) ? true : auto_pause;
  this.init();
  this.draw();
}

/// Reset all intervals.
Simulator.prototype.release = function() {
  if (this.step_timer) clearInterval(this.step_timer);
  if (this.interaction_timer) clearInterval(this.interaction_timer);
  this.pscene.onWorldChange.removeListener(this.draw);
}

Simulator.prototype.pause = function() {
  this.was_autopaused = false;
  if (!this.playing) return;
  clearInterval(this.step_timer);
  this.step_time = null;
  this.playing = false;
}

Simulator.prototype.play = function() {
  if (this.playing) return;
  var self = this;
  self.was_autopaused = false;
  this.step_timer = setInterval(function() {
    self.pscene.step();
    if (self.auto_pause && self.pscene.countAwake() == 0) {
      self.pause();
      self.was_autopaused = true;
    }
  }, this.step_interval);
  this.playing = true;
}

Simulator.prototype.toggle = function() {
  if (this.playing) this.pause();
  else this.play();
}

Simulator.prototype.reset = function() {
  this.pscene.reset();
}

Simulator.prototype.init = function() {
  var self = this;

  // setup debug draw
  this.dbgDraw = new b2DebugDraw();
  this.dbgDraw.SetSprite(this.canvas.getContext("2d"));
  this.dbgDraw.SetDrawScale(this.draw_scale);
  this.dbgDraw.SetXFormScale(0.1);
  this.dbgDraw.SetFillAlpha(0.5);
  this.dbgDraw.SetLineThickness(1.0);
  this.dbgDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);// | b2DebugDraw.e_centerOfMassBit);
  this.pscene.world.SetDebugDraw(this.dbgDraw);
  this.pscene.onWorldChange.addListener(function() { self.draw.apply(self) });

  // setup mouse interaction
  this.mouseDown = false;
  this.mousePoint = new b2Vec2(0,0);
  this.canvas.addEventListener("mousemove", function() {
    self.handleMouseMove.apply(self, arguments);
  }, true);
  this.canvas.addEventListener("mousedown", function() {
    self.mouseDown = true;
  }, true);
  this.canvas.addEventListener("mouseup", function() {
    // if we didn't move a body, it was a normal click and we toggle the playing state
    if (!self.mouseJoint) self.toggle.apply(self);
    self.handleMouseUp.apply(self, arguments);
  }, true);
  this.canvas.addEventListener("dblclick", function() {
    self.pause();
    self.reset();
  }, true);

  // setup timers
  this.interaction_timer = setInterval(function() {
    self.updateInteraction.apply(self);
  }, this.interaction_interval);

  // get position of canvas on screen and update on scrolling to get the mouse position relative to the canvas.
  this.canvas_position = this.getElementPosition(this.canvas);
  window.addEventListener("scroll", function() { self.canvas_position = self.getElementPosition(self.canvas) });
};

Simulator.prototype.getElementPosition = function(el) {
  var x = el.offsetLeft - document.documentElement.scrollLeft,
      y = el.offsetTop - document.documentElement.scrollTop;
  while (el = el.offsetParent) {
    x += el.offsetLeft - el.scrollLeft;
    y += el.offsetTop - el.scrollTop;
  }
  return {x: x, y: y};
}


Simulator.prototype.handleMouseUp = function(evt) {
  this.mouseDown = false;
  if (this.mouseJoint) {
    this.pscene.world.DestroyJoint(this.mouseJoint);
	  this.mouseJoint = null;
	}
}

Simulator.prototype.handleMouseMove = function(evt) {
  this.mousePoint.x = (evt.clientX - this.canvas_position.x) / this.draw_scale;
  this.mousePoint.y = (evt.clientY - this.canvas_position.y) / this.draw_scale;
  if (this.mouseDown && !this.playing) {
    if (this.was_autopaused) this.play();
    else this.pscene.step();
  }
  // draw new mouse position if we would not redraw otherwise
  if (this.draw_pos && !this.mouseDown && !this.playing) this.draw();
}

/** Checks which object is at the passed position, a b2Vec2. Returns the selected body. */
Simulator.prototype.getBodyAtMouse = function() {
  var aabb = new b2AABB();
  var p = this.mousePoint;
	aabb.lowerBound.Set(p.x - 0.001, p.y - 0.001);
	aabb.upperBound.Set(p.x + 0.001, p.y + 0.001);

	var body = null;
  // first search for body
	var filter = function(fixture) {
	  var b = fixture.GetBody();
  	if (b.GetType() != b2Body.b2_staticBody && fixture.GetShape().TestPoint(b.GetTransform(), p)) {
      body = b;
      return false;
    }
    return true;
  }
  this.pscene.world.QueryAABB(filter, aabb);

  return body;
}

/// Updates mouse joint target and creates it if neccessary.
/// If mouse button is not down, it will destroy the joint.
Simulator.prototype.updateInteraction = function() {
  if (this.mouseDown && !this.mouseJoint) {
    var body = this.getBodyAtMouse();
    if (body) {
      var md = new b2MouseJointDef();
      md.bodyA = this.pscene.world.GetGroundBody();
      md.bodyB = body;
      md.target = this.mousePoint;
      md.collideConnected = true;
      md.maxForce = 300.0 * body.GetMass();
      this.mouseJoint = this.pscene.world.CreateJoint(md);
      body.SetAwake(true);
    }
  }
  if (this.mouseJoint) {
    if (this.mouseDown) {
      this.mouseJoint.SetTarget(this.mousePoint);
    } else {
      this.pscene.world.DestroyJoint(this.mouseJoint);
      this.mouseJoint = null;
    }
  }
}

Simulator.prototype.draw = function() {
  if (!this.drawing) return;
  this.pscene.world.DrawDebugData();

  if (this.show_time || this.show_pos) {
    var text = '';
    if (this.show_pos && this.mousePoint) text += ' x='+this.mousePoint.x.toFixed(2) + " y=" + this.mousePoint.y.toFixed(2);
    if (this.show_time) text += ' t=' + this.pscene.getTime().toFixed(2);
    this.ctx.fillStyle = "black";
    this.ctx.fillText(text, 5, 10);
  }
}

s2p.Simulator = Simulator;// Copyright Erik Weitnauer 2014.
/** A wrapper around the b2World class */

Box2D.Common.b2Settings.b2_linearSleepTolerance = 0.1 //0.01;
Box2D.Common.b2Settings.b2_angularSleepTolerance = 20.0 / 180.0 * Math.PI //2.0 / 180.0 * b2Settings.b2_pi;

var PhysicsScene = function(world, dt) {
	this.world = world;
	this.world.curr_time = this.world.curr_time || 0;
	this.world.PushState(); // save initial state for reset
	this.dt = dt || 1/50;
	this.onWorldChange = new ListenerPattern(); // Emits world.curr_time each time the world changed.
	this.emit_changes = true; // set to false to stop updating the world change listeners
}

PhysicsScene.prototype.pushState = function() {
	this.world.PushState();
}

PhysicsScene.prototype.popState = function() {
	this.world.PopState();
	if (this.emit_changes) this.onWorldChange.emit(this.world.curr_time);
}

PhysicsScene.prototype.getState = function() {
	return this.world.GetState();
}

PhysicsScene.prototype.setState = function(state) {
	this.world.SetState(state);
	if (this.emit_changes) this.onWorldChange.emit(this.world.curr_time);
}

PhysicsScene.prototype.reset = function() {
	this.popState();
	this.pushState();
}

PhysicsScene.prototype.getTime = function() {
	return this.world.curr_time;
}

PhysicsScene.prototype.seek = function(t) {
	if (this.world.curr_time > t) this.reset();
	this.simulate(t - this.world.curr_time);
}

PhysicsScene.prototype.clearForces = function() {
	this.world.ClearForces();
}

/// dt is optional, returns dt.
PhysicsScene.prototype.step = function(dt) {
	dt = dt || this.dt;
	try { // in Box2D.js line 5218, we sometimes have proxyA being undefined
		this.world.Step(dt, 10, 10);
	} catch(err) {
		console.log('caught error', err, 'during Box2D simulation step');
		console.log('trying again after finding new contacts...');

		var broadPhase = this.world.m_contactManager.m_broadPhase;
		this.forEachBody(function(body) {
		  for (var f = body.m_fixtureList; f; f = f.m_next) {
		  	if (!f.m_proxy) {
		  		console.log(body, f, 'has no m_proxy set. Creating it now...');
      		f.CreateProxy(broadPhase, body.m_xf);
      	}
     	}
  	});

		//this.world.m_contactManager.FindNewContacts();
		//var curr_time = this.world.curr_time;
		//this.reset();
		//this.simulate(curr_time+dt);
		this.step(dt);
	}
  this.world.curr_time += dt;
  if (this.emit_changes) this.onWorldChange.emit(this.world.curr_time);
  return dt;
}

/// Makes as many steps of this.dt as needed to reach time.
PhysicsScene.prototype.simulate = function(time) {
  var t = 0;
  while (t+this.dt<time) t += this.step();
  var rest = time-t;
  if (rest > 0.001) this.step(rest);
}

/// Simulates until all bodies sleep or max_time (default: Inifinity) is reached. Returns time.
PhysicsScene.prototype.simulateUntilSleep = function(max_time) {
	var max_time = max_time || Infinity;
	var t = 0;
	while (t<=max_time && this.countAwake() > 0) t += this.step();
	return t;
}

/// It saves the world state, calls the start_callback, simulates the world for the passed
/// time, calls the end_callback and restores the previous world state. Returns the value
/// returned by end_callback.
/// Temporarily disables the onWorldChange events.
/// The start_callback can be used to, e.g., apply an impulse. It can also be null.
/// If untilSleep is passed as true, the simulation might stop before `time`, if all bodies in
/// the scene are at rest.
PhysicsScene.prototype.analyzeFuture = function(time, start_callback, end_callback, untilSleep) {
	if (time < 0) throw "You are mistaking the past for the future."
	var old_emit_changes = this.emit_changes;
	this.emit_changes = false;
	this.pushState();
	if (start_callback) start_callback();
	if (time > 0) {
		if (untilSleep) this.simulateUntilSleep(time);
		else this.simulate(time);
	}
	var res = end_callback();
	this.popState();
	this.emit_changes = old_emit_changes;
	return res;
};

/// Calls the passed function for all bodies that have a master_object.
PhysicsScene.prototype.forEachBody = function(f) {
  for (var b = this.world.m_bodyList; b; b = b.m_next) {
  	if (b.master_obj) f(b);
  }
}

/// Calls the passed function for all dynamic bodies that have a master_object.
PhysicsScene.prototype.forEachDynamicBody = function(f) {
  for (var b = this.world.m_bodyList; b; b = b.m_next) {
  	if (b.GetType() == b2Body.b2_dynamicBody) f(b);
 	}
}

/// Returns the total kinetic energy of all bodies.
PhysicsScene.prototype.getKineticEnergy = function() {
	var energy = 0;
	this.world.forEachDynamicBody(function(b) {
		energy += 0.5 * (b.m_I*b.m_angularVelocity*b.m_angularVelocity
		                +b.m_mass*b.m_linearVelocity.Length()*b.m_linearVelocity.Length());
	});
	return energy;
}

/// Returns the distance the body has travelled between the passed (old) transformation
/// and the current transformation. For circles, the euclidian distance of the center is
/// returned. For other shapes, the mean distance of all corners' distances is returned.
/// If no transformation is passed, the transformation of the previous body state on its
/// bodystates stack is used.
PhysicsScene.prototype.getBodyDistance = function(body, xf) {
	xf = xf || body.bodystates[body.bodystates.length-1].m_xf
  if (body.m_fixtureList.m_shape.GetType() == b2Shape.e_circleShape) {
    var d = body.m_xf.position.Copy();
    d.Subtract(xf.position);
    return d.Length();
  } else {
    return this.meanPointDistance(body.m_fixtureList.m_shape.GetVertices(), body.m_xf, xf);
  }
}

/// Returns the mean distance of the passed points between their position in
/// the first and the second transformation.
/// This method is used by the getBodyDistance method.
PhysicsScene.prototype.meanPointDistance = function(points, xf1, xf2) {
  var dist = 0;
  for (var i=0; i<points.length; i++) {
    var p = points[i];
    var d = p.Transformed(xf1);
    d.Subtract(p.Transformed(xf2));
    dist += d.Length();
  }
  return dist / points.length;
}

/// Wakes up all bodies.
PhysicsScene.prototype.wakeUp = function () {
	for (var b = this.world.m_bodyList; b; b = b.m_next) b.SetAwake(true);
}

/// Returns the number of dynamic objects that are awake.
PhysicsScene.prototype.countAwake = function() {
	var count = 0;
	this.forEachDynamicBody(function(b) { if (b.IsAwake()) count++ });
	return count;
}

var ListenerPattern = function() {
	this.listeners = [];
}
ListenerPattern.prototype.addListener = function(l) { this.listeners.push(l) }
ListenerPattern.prototype.removeListener = function(l) {
	var i=this.listeners.indexOf(l);
	if (i>=0) Array.remove(this.listeners, l);
}
ListenerPattern.prototype.removeAll = function() { this.listeners = [] }
ListenerPattern.prototype.emit = function() {
	for (var i = 0; i < this.listeners.length; i++) {
		this.listeners[i].apply(this.listeners[i], arguments);
	}
}

s2p.PhysicsScene = PhysicsScene;// Copyright Erik Weitnauer 2014.
var SVGSceneParser = (function() {
  var PREFIX = 's2p-';
  var pub = {};

  /// Load the content of given URL.
  var ajaxGetUrl = function(url) {
    var AJAX;
    if(window.XMLHttpRequest){AJAX=new XMLHttpRequest();}
    else{AJAX=new ActiveXObject('Microsoft.XMLHTTP');}
    if(AJAX){
      AJAX.open('GET',url,false);
      AJAX.send(null);
      return AJAX.responseText;
    }
    return null;
  }
  pub.ajaxGetUrl = ajaxGetUrl;

  // Parse an XML string, return as DOM tree.
  var parseXml = function(xml) {
    if (window.DOMParser) {
      var parser = new DOMParser();
      //return parser.parseFromString(xml, 'image/svg+xml');
      return parser.parseFromString(xml, 'image/svg+xml');
    } else {
      xml = xml.replace(/<!DOCTYPE svg[^>]*>/, '');
      var xmlDoc = new ActiveXObject('Microsoft.XMLDOM');
      xmlDoc.async = 'false';
      xmlDoc.loadXML(xml);
      return xmlDoc;
    }
  }
  pub.parseXml = parseXml;

  /// Reads all styles set for a node and returns them as a hash.
  var readStyle = function(node, scaling) {
    var s = {};
    for (var i=0; i<node.style.length; ++i) {
      var key = node.style.item(i);
      s[key] = node.style.getPropertyValue(key);
    }
    return s;
  }

  /// Returns the scaling that is done by the passed svg transformation.
  var extract_scaling = function(tf) {
    return (Point.len(tf.a, tf.b) + Point.len(tf.c, tf.d)) / 2
  }

  /// Loads the shapes in the scene from the contents of an svg file (passed as string).
  /// The default pixels_per_unit is 100.
  var parseFile = function(file_url, pixels_per_unit) {
    pixels_per_unit = pixels_per_unit || 100;
    //console.log('parsing', file_url);
    var content = ajaxGetUrl(file_url);
    var svg_dom = parseXml(content);
    if (!svg_dom) throw 'Error parsing ' + content;

    // WORKAROUND 1
    // normally we would use the svg dom tree directly, but due to this bug in Firefox
    // (https://bugzilla.mozilla.org/show_bug.cgi?id=756985),
    // we need to add it to the main dom tree if we want to use getCTM() dom function.
    var root = append_svg_to_dom(svg_dom, "hidden_svg_div");
    var shapes = [];

    // the stroke-width must be multiplied with the scaling of the object to get the
    // actual stroke width
    var scale_stroke_width = function(node, scale) {
      var sw = node.style['stroke-width'];
      if (!sw) return;
      node.style['stroke-width'] = sw.replace(/^[0-9]*\.?[0-9]+/, function(x) {return Number(x)*scale;});
    }

    /// parse all rects
    var rects = root.getElementsByTagName('rect');
    for (var i=0; i<rects.length; i++) {
      var rect_node = rects[i];
      var shape = Polygon.fromSVGRect(rect_node);
      shape.svg_transform = rect_node.getCTM();
      shape.style = readStyle(rect_node);
      scale_stroke_width(shape, extract_scaling(shape.svg_transform));
      shapes.push(shape);
    }

    // the frame is the biggest rect in the scene
    var frame = find_biggest(shapes);
    frame.is_frame = true;
    shapes = shapes.filter(function(o) { return !o.is_frame});

    var paths = root.getElementsByTagName('path');
    for (var i=0; i<paths.length; i++) {
      var path_node = paths[i];
      // circles are saved as paths in old Inkscape versions, so the path_node
      // is either a circle or a real path
      var shape = Circle.fromSVGPath(path_node, false) ||
                Polygon.fromSVGPath(path_node, 1, false);
      if (shape instanceof Polygon) {
        shape.merge_vertices({min_dist: 1, min_vertex_count: 2});
      }
      shape.svg_transform = path_node.getCTM();
      shape.style = readStyle(path_node);
      scale_stroke_width(shape, extract_scaling(shape.svg_transform));
      shapes.push(shape);
    }

    var circles = root.getElementsByTagName('circle');
    for (var i=0; i<circles.length; i++) {
      var path_node = circles[i];
      var shape = Circle.fromSVGCircle(path_node);
      shape.svg_transform = path_node.getCTM();
      shape.style = readStyle(path_node);
      scale_stroke_width(shape, extract_scaling(shape.svg_transform));
      shapes.push(shape);
    }

    // set `movable` property for all object that have no black stroke color
    shapes.forEach(function(s) {
      var color = s.style.stroke;
      s.movable = !(color == "#000000" || color == "#000" || color == "black" || color == "rgb(0, 0, 0)");
    });

    apply_transformations([frame], 0, 0, 1, root);
    var s  = 100/Math.abs(frame.pts[0].x-frame.pts[1].x)
       ,dx = -Math.min(frame.pts[0].x, frame.pts[1].x)*s
       ,dy = -Math.min(frame.pts[0].y, frame.pts[2].y)*s
    apply_transformations(shapes, dx, dy, s, root);
    apply_transformations([frame], dx, dy, s, root);

    return new SVGScene(shapes, frame, pixels_per_unit);
  }
  pub.parseFile = parseFile;

  /// Applies the svg transformation to each object.
  var apply_transformations = function(shapes, dx, dy, s, svg) {
    var svg_pt = svg.createSVGPoint(0,0);
    shapes.forEach(function(shape) {
      var transform = function(p) {
        svg_pt.x = p.x; svg_pt.y = p.y;
        if (shape.svg_transform) svg_pt = svg_pt.matrixTransform(shape.svg_transform);
        p.x = s*svg_pt.x+dx; p.y = s*svg_pt.y+dy;
      }
      if (shape instanceof Circle) {
        var c = shape.centroid(); transform(c);
        shape.x = c.x; shape.y = c.y;
        shape.r *= s;
        if (shape.svg_transform) shape.r *= Math.abs(shape.svg_transform.a);
      }
      else if (shape instanceof Polygon) shape.pts.forEach(transform);
      else throw "Unkown object type";
      delete shape.svg_transform;
    });
  }

  var find_biggest = function(objs) {
    var max_area = 0, biggest = null;
    for (var i=0; i<objs.length; i++) {
      var scaling = objs[i].svg_transform.a;
      var area = Math.abs(objs[i].area()*scaling*scaling);
      if (area > max_area) { max_area = area; biggest = objs[i]}
    };
    return biggest;
  }

  /// Will append the passed dom structure to the main document inside the div
  /// with the passed id. If the div does not exist it is created and styled
  /// so its invisible.
  var append_svg_to_dom = function(svg_dom, parent_id) {
    var parent = document.getElementById(parent_id);
    if (!parent) {
      parent = document.body.appendChild(document.createElement('div'));
      parent.setAttribute("id", parent_id);
      // it is important for Chromium 18 to set the style this way and not by using parent.style.xxx
      // because otherwise evaluating an XPathExpression on the main dom tree will cause an
      // INVALID_STATE_ERR: DOM Exception 11
      parent.setAttribute("style", "position:absolute;width:1px;height:1px;overflow:hidden;left:-10px;");
    } else {
      var child;
      while (child = parent.childNodes[0]) { parent.removeChild(child); }
    }
    return parent.appendChild(svg_dom.rootElement);
  }

  return pub;
})();


SVGScene = function(shapes, frame, pixels_per_unit) {
  this.shapes = shapes || []; // may contain polygons or circles
  this.frame = frame;
  this.shapes.push(frame);
  this.setIds();
  this.width = 100;
  this.height = 100;
  this.friction = 0.3;
  this.restitution = 0.1;
  this.pixels_per_unit = pixels_per_unit;
  this.moveToOrigin();
}

SVGScene.prototype.adjustStrokeWidth = function(width) {
  var reg_float = /^[0-9]*\.?[0-9]+/;
  for (var i=0; i<this.shapes.length; i++) {
    var shape = this.shapes[i];
    var stroke_width = 1;
    if (reg_float.test(shape.style['stroke-width'])) {
      stroke_width = Number(reg_float.exec(shape.style['stroke-width'])[0]);
    }
    var bb = shape.bounding_box();
    var scale_x = (bb.width + stroke_width) / (bb.width + width);
    var scale_y = (bb.height + stroke_width) / (bb.height + width);
    if (shape instanceof Polygon && shape.id !== '|') {
      shape.pts.forEach(function(p) { p.x *= scale_x; p.y *= scale_y });
    } else if (shape instanceof Circle) {
      shape.r = shape.r * scale_x;
    }
    shape.style['stroke-width'] = width;
  }
}

SVGScene.prototype.setIds = function() {
  for (var i=0; i<this.shapes.length; i++) {
    if (this.shapes[i].movable) this.shapes[i].id = i;
    else if (this.shapes[i] == this.frame) this.shapes[i].id = '|'
    else this.shapes[i].id = '_';
  }
}

/// Centers all polygon shapes onto the origin and saves their original centers in x, y.
/// Also sets rot to 0.
SVGScene.prototype.moveToOrigin = function() {
  for (var i=0; i<this.shapes.length; i++) {
    var shape = this.shapes[i];
    if (!(shape instanceof Polygon)) continue;
    var pos = shape.centroid();
    shape.pts.forEach(function(p) { p.Sub(pos) });
    shape.x = pos.x;
    shape.y = pos.y;
    shape.rot = 0;
  }
}

SVGScene.prototype.renderInSvg = function(doc, parent, x, y, scale, show_numbers) {
  var g = doc.createElementNS('http://www.w3.org/2000/svg','g');
  g.setAttribute('transform', 'translate('+(x)+','+(y)+') scale('+scale+')');
  parent.appendChild(g);
  var rect = doc.createElementNS('http://www.w3.org/2000/svg','rect');
  for (var i = 0; i < this.shapes.length; i++) {
    var shape = this.shapes[i];
    var svg_obj = shape.renderInSvg(document, g);
    for (var s in shape.style) svg_obj.style.setProperty(s, shape.style[s]);
    if (show_numbers && this.shapes[i].movable) {
      d3.select(parent).append('text').style('fill', 'black')
        .attr('x', shape.x*scale).attr('y', shape.y*scale)
        .attr('text-anchor', 'middle')
        .attr('dominant-baseline', 'central').text(i);
    }
  }
}

s2p.SVGSceneParser = SVGSceneParser;
/// Written by Erik Weitnauer, 2013.
/**
PhyscisOracle:
This class provides all functionality of the "physics black box" to the PBP Interpreter. It has the following methods:

- observeCollisions(): gives back an array of collision events {a, b, dv, t} where a is the body moving faster towards the other (the active one)
- getTouchGroups(): gives back an array of arrays of dyn. bodies that directly or indirectly touch each other
- getTouchedBodies(body): gives back an array of bodies directly touched by the passed body, potentially including the ground.
*/
/// Pass a b2World.
var PhysicsOracle = function(physics_scene) {
	this.pscene = physics_scene;
  this.pscene.onWorldChange.addListener(this.onWorldChange.bind(this));
	this.contact_listener = new PhysicsOracle.ContactListener(this);
  this.curr_state = "0"; // null if unknown
  this.states = {'0'    : {time: 0, pstate: null},
                 'start': {time: 0.08, pstate: null},
                 'end'  : {time: 'end', pstate: null}};
}

/// The state can be one of the ones defined in this.states. Each state gets saved the first time
/// it is reached so the second time, no simulation is neccessary.
PhysicsOracle.prototype.gotoState = function(state) {
  if (this.curr_state === state) return;
  if (!(state in this.states)) {
    this.curr_state = null;
    throw 'unknown state "' + state + '"';
  }
  var s = this.states[state];
  if (s.pstate) this.loadPhysicsState(s.pstate);
  else {
    if (this.states[state].time == 'end') this.pscene.simulateUntilSleep(12);
    else this.pscene.seek(this.states[state].time);
    this.savePhysicsState(state);
  }
  this.curr_state = state;
}

PhysicsOracle.prototype.useCurrAsInitialState = function() {
  this.pscene.world.curr_time = 0;
  this.pscene.world.PushState();
  this.curr_state = '0';
  d3.values(this.states).forEach(function(state) { state.pstate = null });
  this.pscene.reset();
}

/// Get the current state of the physics simulation and save in this.states.
PhysicsOracle.prototype.savePhysicsState = function(state) {
  this.states[state].pstate = this.pscene.getState();
}

/// Revert to a previously recorded state of the physics world.
PhysicsOracle.prototype.loadPhysicsState = function(pstate) {
  this.pscene.setState(pstate);
}

/// It saves the world state, calls the start_callback, simulates the world for the passed
/// time, calls the end_callback and restores the previous world state. Returns the value
/// returned by end_callback. Since pscene.analyzeFuture temporarily deactivates the worldChanged
/// callbacks, the PhysicsOracle will still be in the same state after simulation, as it was before.
/// The start_callback can be used to, e.g., apply an impulse. It can also be null.
/// If untilSleep is passed as true, the simulation might stop before `time`, if all bodies in
/// the scene are at rest.
PhysicsOracle.prototype.analyzeFuture = function(time, start_callback, end_callback, untilSleep) {
  return this.pscene.analyzeFuture(time, start_callback, end_callback, untilSleep);
}

/// Called when the world changed, calls synchShapes and sets curr_state to null.
PhysicsOracle.prototype.onWorldChange = function() {
  this.curr_state = null;
  this.synchShapes();
}

/// Calls synch_to_phys for every body's master shape object. This updates the x, y and rot attributes of
/// the shapes.
PhysicsOracle.prototype.synchShapes = function() {
  this.pscene.forEachBody(function(b) { b.master_obj.synch_to_phys() });
}

PhysicsOracle.prototype.isStatic = function(body) {
  return body.m_type == b2Body.b2_staticBody;
}

/// Applies an impulse to the center of the passed object.
/// Strength can either be a float (it is multiplied with the direction to get the
/// input) or a string ('small', 'medium' or 'large' - the strength is set to
/// 0.5, 1 or 1.5 of the body's mass). Dir can either be a b2Vec2 or a string
/// ('left', 'right', 'up' or 'down').
PhysicsOracle.prototype.applyCentralImpulse = function(body, dir, strength) {
  /// Impulse is a b2Vec, where its direction is the direction of the impulse and
  /// its length is the strength of the impulse in kg*(m/s) which is mass*velocity.
  /// Point is a b2Vec and is the point to which the impulse is applied in body coords.
  var applyImpulse = function(body, impulse, point) {
    var p = point.Copy(); p.Add(body.m_sweep.c)
    body.ApplyImpulse(impulse, p);
  }
  var strength_map = {'small': 0.5, 'medium': 1.0, 'large': 1.5};
  var dir_map      = {'left': new b2Vec2(-1,0), 'right': new b2Vec2(1,0),
                        'up': new b2Vec2(0,1),   'down': new b2Vec2(0,-1)};
  if (typeof(strength) == 'string') strength = strength_map[strength] * body.m_mass;
  if (typeof(dir) == 'string') dir = dir_map[dir];
  var impulse = dir.Copy();
  impulse.Multiply(strength);
  applyImpulse(body, impulse, new b2Vec2(0,0));
}

/// Returns all objects grouped by touch. E.g "A  BC" will be returned as [[A], [B,C]]. Only
/// regards dynamic bodies.
PhysicsOracle.prototype.getTouchGroups = function() {
  var touches = [], bodies = [];
  this.pscene.forEachDynamicBody(function(b) { bodies.push(b) });
  // link all dynamic bodies which touch
  for (var c = this.GetContactList(); c; c=c.m_next) {
    if (!c.IsTouching()) continue;
    var a = c.m_fixtureA.m_body, b = c.m_fixtureB.m_body;
    if (a.GetType() !== b2Body.b2_dynamicBody ||
        b.GetType() !== b2Body.b2_dynamicBody) continue;
    touches.push([a, b]);
  }
  return this.groupLinkedNodes(bodies, touches);
}

/// Returns an object {body: b2Body, dist: float} or false, if there
/// is no other body in the scene. Will only consider dynamic bodies.
PhysicsOracle.prototype.getClosestBodyWithDist = function(body) {
  var res = { body: null, dist: Infinity };
  this.pscene.forEachDynamicBody(function(other) {
    if (other === body) return;
    var dist = body.distance(other);
    if (dist < res.dist) {
      res.body = other;
      res.dist = dist;
    }
  });
  if (res.body === null) return null;
  return res;
}

/// Returns a list with all touched bodies, possibly including the ground or the frame.
/// Each touched body is only in the list once, even if touched at several places.
PhysicsOracle.prototype.getTouchedBodies = function(body) {
  var res = [];
  var gb = body.m_world.m_groundBody;
  for (var c = body.m_world.GetContactList(); c; c=c.m_next) {
    if (!c.IsTouching()) continue;
    var a = c.m_fixtureA.m_body, b = c.m_fixtureB.m_body;
    if (a != body && b != body) continue;
    if (a == gb || b == gb) continue;
    a = (a == body ? b : a);
    if (res.indexOf(a) == -1) res.push(a);
  }
  return res;
}

/// Returns a list of {body: touched_body, pts: [world_pos]} objects. If an object is
/// touched at several places, it might be in the list several times.
PhysicsOracle.prototype.getTouchedBodiesWithPos = function(body) {
  var res = [];
  var gb = body.m_world.m_groundBody;
  var wm = new Box2D.Collision.b2WorldManifold();
  for (var c = body.m_world.GetContactList(); c; c=c.m_next) {
    if (!c.IsTouching()) continue;
    var a = c.m_fixtureA.m_body, b = c.m_fixtureB.m_body;
    if (a != body && b != body) continue;
    if (a == gb || b == gb) continue;
    c.GetWorldManifold(wm);
    var pts = wm.m_points.slice(0, c.m_manifold.m_pointCount);
    res.push({body: (a == body ? b : a), pts: pts});
  }
  return res;
}

/// Returns all objects grouped by vicinity. E.g "A    B C" will be returned as [[A], [B,C]]
/// if dist(A, B) > max_dist and dist(B,C) is <= max_dist. All static objects are ignored.
/// If no bodies are passed, all bodies in the scene are used.
PhysicsOracle.prototype.getSpatialGroups = function(max_dist, bodies) {
  var links = [];
  if (!bodies) {
    bodies = [];
    this.pscene.forEachDynamicBody(function(b) { bodies.push(b) });
  }
  for (var i=0; i<bodies.length-1; i++) for (var j=i+1; j<bodies.length; j++) {
    if (bodies[i].distance(bodies[j]) <= max_dist) links.push([bodies[i], bodies[j]]);
  };
  return this.groupLinkedNodes(bodies, links);
}

/// Returns the nodes in groups where each group contains all nodes between which there is
/// a path of links.
PhysicsOracle.prototype.groupLinkedNodes = function(nodes, links) {
  var res=[];
  for (var i=0; i<nodes.length; i++) {
    res.push([nodes[i]]);
    nodes[i]._ew_group_ = i;
  }
  for (var i=0; i<links.length; i++) {
    var n1 = links[i][0], n2 = links[i][1];
    var g1 = n1._ew_group_, g2 = n2._ew_group_;
    if (g1 == g2) continue;
    // put all objects from group g2 into g1
    for (var j=0; j<res[g2].length; j++) {
      var n3 = res[g2][j];
      n3._ew_group_ = g1;
      res[g1].push(n3);
    }
    res[g2] = [];
  }
  for (var i=0; i<nodes.length; i++) delete nodes[i]._ew_group_;
  return res.filter(function(x){return x.length});
}

/// Gives back an array of collision events {a, b, dv, t} where a is the 'hitter' and b the
/// 'hit' body.
PhysicsOracle.prototype.observeCollisions = function() {
	var old_cl = this.pscene.world.m_contactManager.m_contactListener;
	this.pscene.world.SetContactListener(this.contact_listener);
	this.collisions = [];
  var thiz = this;

  this.analyzeFuture(12, null, function() {
    thiz.pscene.world.SetContactListener(old_cl);
    thiz.collisions = PhysicsOracle.mergeCollisions(thiz.collisions, 0);
    // save current state as end state, if we didn't cache it yet
    if (!thiz.states.end.pstate) thiz.savePhysicsState('end');
  }, true);

  return this.collisions;
}

/// Merges collision of any body pair that are temporally closer than `max_dt` default: 0.25 s.
/// It also removes any collision that happened before `min_t` default: 0.1 s (this is useful
/// since often objects that are supposed to lie on the ground are hitting the ground in the
/// first 0.1 seconds).
PhysicsOracle.mergeCollisions = function(collisions, min_t, max_dt) {
  var res = [];
  if (typeof(max_dt) == 'undefined') max_dt = 0.25;
  if (typeof(min_t)  == 'undefined') min_t  = 0.1;
  for (var i=0; i<collisions.length; i++) {
    var c = collisions[i];
    if (c.t < min_t) continue;
    var r = res[res.length-1];
    if (r && ((r.a == c.a && r.b == c.b) || (r.a == c.b && r.b == c.a))
          && (Math.abs(r.t - c.t) <= max_dt)) {
      r.dv = Math.max(r.dv, c.dv);
    } else {
      res.push(c);
    }
  }
  return res;
}

PhysicsOracle.ContactListener = function(parent) {
	var wm = new Box2D.Collision.b2WorldManifold();
	this.BeginContact = function (contact) {}
  this.EndContact = function (contact) {}
  /// for a new or changed contact, save the delta velocities in normal direction for all
  /// contact points
  this.PreSolve = function (contact, oldManifold) {
  	// don't do anything if we already know this contact and it had the same
  	// number of points in its manifold last time already
  	if (!contact.IsTouching()) {
    	contact.pointCount = 0;
    	contact.process = false;
    	return;
  	}
  	if (contact.pointCount && contact.pointCount == contact.m_manifold.m_pointCount) {
    	contact.process = false;
    	return;
 	 	}
  	contact.pointCount = contact.m_manifold.m_pointCount;
  	contact.process = true;
  	var bodya = contact.m_fixtureA.m_body, bodyb = contact.m_fixtureB.m_body;
  	contact.GetWorldManifold(wm);
  	var max_vel_a = 0, max_vel_b = 0;
  	var norm = wm.m_normal;

  	// If a rectangle lies on the ground, you pick up one corner, tilt it and let it fall
  	// back to the ground then you'll have two contact points, but only in one of them
  	// there will be a relative speed (on the side that was lifted). We are interested the
  	// highest objects speeds in all contact points.
  	// CAUTION: wm.m_points might contain more points than are actually set,
  	//          we can only use the first contact.m_manifold.m_pointCount points!
  	for (var i=0; i<contact.m_manifold.m_pointCount; i++) {
    	var vel_a = bodya.GetLinearVelocityFromWorldPoint(wm.m_points[i]);
    	var vel_b = bodyb.GetLinearVelocityFromWorldPoint(wm.m_points[i]);
    	vel_a = vel_a.x*norm.x + vel_a.y*norm.y;
    	vel_b = vel_b.x*norm.x + vel_b.y*norm.y;
    	if (Math.abs(vel_a) > Math.abs(max_vel_a)) max_vel_a = vel_a;
    	if (Math.abs(vel_b) > Math.abs(max_vel_b)) max_vel_b = vel_b;
  	}
  	contact.vel_a = max_vel_a;
  	contact.vel_b = max_vel_b;
	}
	/// if the delta speed is high enough,
	this.PostSolve = function (contact, impulse) {
    if (!contact.process) return;
	  var dv = Math.abs(contact.vel_a-contact.vel_b);
  	if (dv > 0.5) {
    	var bodya = contact.m_fixtureA.m_body, bodyb = contact.m_fixtureB.m_body;
      var world = bodya.m_world;
    	if (Math.abs(contact.vel_a) > Math.abs(contact.vel_b)) {
      	//console.log(bodya.master_obj.id, 'hits', bodyb.master_obj.id, 'with', dv, 'at', world.curr_time);
      	parent.collisions.push({a: bodya, b: bodyb, dv:dv, t: world.curr_time});
    	} else {
      	//console.log(bodyb.master_obj.id, 'hits', bodya.master_obj.id, 'with', dv, 'at', world.curr_time);
      	parent.collisions.push({a:bodyb, b:bodya, dv:dv, t: world.curr_time});
    	}
    }
  }
}

s2p.PhysicsOracle = PhysicsOracle;
  return s2p;
})();