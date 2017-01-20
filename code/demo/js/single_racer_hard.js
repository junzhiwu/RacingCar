var canvas, ctx;

// A 2D vector utility
var Vec = function(x, y) {
  this.x = x;
  this.y = y;
}
Vec.prototype = {
  // utilities
  dist_from: function(v) { return Math.sqrt(Math.pow(this.x-v.x,2) + Math.pow(this.y-v.y,2)); },
  length: function() { return Math.sqrt(Math.pow(this.x,2) + Math.pow(this.y,2)); },
  // new vector returning operations
  add: function(v) { return new Vec(this.x + v.x, this.y + v.y); },
  sub: function(v) { return new Vec(this.x - v.x, this.y - v.y); },
  rotate: function(a) {  // CLOCKWISE
    return new Vec(this.x * Math.cos(a) + this.y * Math.sin(a),
                   -this.x * Math.sin(a) + this.y * Math.cos(a));
  },
  // in place operations
  scale: function(s) { this.x *= s; this.y *= s; },
  normalize: function() { var d = this.length(); this.scale(1.0/d); },
  // scalar returning operations
  project: function(v) { return (this.x*v.x+this.y*v.y)/v.length(); },
  dist_from_seg: function(v1,v2) {
    var A = this.x - v1.x, B = this.y - v1.y;
    var C = v2.x - v1.x, D = v2.y - v1.y;
    var dot = A * C + B * D;
    var len_sq = C * C + D * D;
    var param = -1;
    if (len_sq != 0) param = dot / len_sq;
    var xx, yy;
    if (param < 0) { xx = v1.x; yy = v1.y;}
    else if (param > 1) { xx = v2.x; yy = v2.y; }
    else { xx = v1.x + param * C; yy = v1.y + param * D;}
    var dx = this.x - xx, dy = this.y - yy;
    return Math.sqrt(dx * dx + dy * dy);
  },
  angle: function(v){
    return Math.acos((v.y*this.y + v.x*this.x)/(v.length()*this.length()));
  }
}

// line intersection helper function: does line segment (p1,p2) intersect segment (p3,p4) ?
var line_intersect = function(p1,p2,p3,p4) {
  var denom = (p4.y-p3.y)*(p2.x-p1.x)-(p4.x-p3.x)*(p2.y-p1.y);
  if(denom===0.0) { return false; } // parallel lines
  var ua = ((p4.x-p3.x)*(p1.y-p3.y)-(p4.y-p3.y)*(p1.x-p3.x))/denom;
  var ub = ((p2.x-p1.x)*(p1.y-p3.y)-(p2.y-p1.y)*(p1.x-p3.x))/denom;
  if(ua>0.0&&ua<1.0&&ub>0.0&&ub<1.0) {
    var up = new Vec(p1.x+ua*(p2.x-p1.x), p1.y+ua*(p2.y-p1.y));
    return {ua:ua, ub:ub, up:up}; // up is intersection point
  }
  return false;
}

var line_point_intersect = function(p1,p2,p0,rad) {
  var v = new Vec(p2.y-p1.y,-(p2.x-p1.x)); // perpendicular vector
  var d = Math.abs((p2.x-p1.x)*(p1.y-p0.y)-(p1.x-p0.x)*(p2.y-p1.y));
  d = d / v.length();
  if(d > rad) { return false; }

  v.normalize();
  v.scale(d);
  var up = p0.add(v);
  if(Math.abs(p2.x-p1.x)>Math.abs(p2.y-p1.y)) {
    var ua = (up.x - p1.x) / (p2.x - p1.x);
  } else {
    var ua = (up.y - p1.y) / (p2.y - p1.y);
  }
  if(ua>0.0&&ua<1.0) {
    return {ua:ua, up:up};
  }
  return false;
}

// Wall is made up of two points
var Wall = function(p1, p2) {
  this.p1 = p1;
  this.p2 = p2;
}

var Route = function(p1, p2) {
  this.p1 = p1;
  this.p2 = p2;
}

// World object contains many agents and walls
var util_add_box = function(lst, x, y, w, h) {
  lst.push(new Wall(new Vec(x,y), new Vec(x+w,y)));
  lst.push(new Wall(new Vec(x+w,y), new Vec(x+w,y+h)));
  lst.push(new Wall(new Vec(x+w,y+h), new Vec(x,y+h)));
  lst.push(new Wall(new Vec(x,y+h), new Vec(x,y)));
};

var util_add_hex = function(lst, x, y, s) {
  lst.push(new Wall(new Vec(x+s/2,y), new Vec(x+s*3/2,y)));
  lst.push(new Wall(new Vec(x+s*3/2,y), new Vec(x+s*2,y+s*1.732/2)));
  lst.push(new Wall(new Vec(x+s*2,y+s*1.732/2), new Vec(x+s*3/2,y+s*1.732)));
  lst.push(new Wall(new Vec(x+s*3/2,y+s*1.732), new Vec(x+s*1/2,y+s*1.732)));
  lst.push(new Wall(new Vec(x+s*1/2,y+s*1.732), new Vec(x, y+s*1.732/2)));
  lst.push(new Wall(new Vec(x,y+s*1.732/2), new Vec(x+s/2, y)));
};

var util_add_broken_hex = function(lst, x, y, s) {
  lst.push(new Wall(new Vec(x+s/2,y), new Vec(x+s*3/4,y)));
  lst.push(new Wall(new Vec(x+s*5/4,y), new Vec(x+s*3/2,y)));

  lst.push(new Wall(new Vec(x+s*3/2,y), new Vec(x+s*2,y+s*1.732/2)));
  //lst.push(new Wall(new Vec(x+s*3/2,y), new Vec(x+s*13/8,y+s*1.732/2/4)));
  //lst.push(new Wall(new Vec(x+s*15/8,y+s*1.732/2*3/4), new Vec(x+s*2,y+s*1.732/2)));

  lst.push(new Wall(new Vec(x+s*2,y+s*1.732/2), new Vec(x+s*3/2,y+s*1.732)));
  //lst.push(new Wall(new Vec(x+s*2,y+s*1.732/2), new Vec(x+s*15/8,y+s*1.732/2*5/4)));
  //lst.push(new Wall(new Vec(x+s*13/8,y+s*1.732/2*7/4), new Vec(x+s*3/2,y+s*1.732)));

  lst.push(new Wall(new Vec(x+s*3/2,y+s*1.732), new Vec(x+s*5/4,y+s*1.732)));
  lst.push(new Wall(new Vec(x+s*3/4,y+s*1.732), new Vec(x+s*1/2,y+s*1.732)));

  lst.push(new Wall(new Vec(x+s*1/2,y+s*1.732), new Vec(x+s*3/8, y+s*1.732*7/8)));
  lst.push(new Wall(new Vec(x+s/8, y+s*1.732*5/8), new Vec(x, y+s*1.732/2)));

  lst.push(new Wall(new Vec(x,y+s*1.732/2), new Vec(x+s/8,y+s*1.732*3/8)));
  lst.push(new Wall(new Vec(x+s*3/8,y+s*1.732/8), new Vec(x+s/2, y)));
};

var util_add_route = function(lst, x, y){
  lst.push(new Vec(x,y));
};

// item is circle thing on the floor that agent can interact with (see or eat, etc)
var Item = function(x, y, type) {
  this.p = new Vec(x, y); // position
  this.type = type;
  this.rad = 10; // default radius
  this.age = 0;
  this.cleanup_ = false;
};

var World = function() {
  this.agents = [];
  this.W = canvas.width;
  this.H = canvas.height;

  this.clock = 0;

  // set up walls in the world
  this.walls = [];
  var pad = 10;
  //util_add_box(this.walls, pad, pad, this.W-pad*2, this.H-pad*2);
  //util_add_box(this.walls, 100, 100, 500, 300); // inner walls
  util_add_box(this.walls, 0, 0, this.W, this.H);

  util_add_hex(this.walls, 35, 140, 125);
  util_add_hex(this.walls, 415, 140, 125);
  util_add_hex(this.walls, 225, 30, 125);
  util_add_hex(this.walls, 225, 250, 125);
  util_add_broken_hex(this.walls, 0, -125*1.732/2, 125);
  util_add_broken_hex(this.walls, this.W-125*2, this.H-125*1.732/2, 125);

  // TODO: set up suggested routes
  this.routes = [];
  util_add_route(this.routes, 70, 50);
   util_add_route(this.routes, 630,50);
   util_add_route(this.routes, 650,70);
   util_add_route(this.routes, 650,430);
   util_add_route(this.routes, 630,450);
   util_add_route(this.routes, 70,450);
   util_add_route(this.routes, 50,430);
   util_add_route(this.routes, 50,70);
   util_add_route(this.routes, 70,50);
  /*util_add_route(this.routes, 50, 50);
  util_add_route(this.routes, 650,50);
  util_add_route(this.routes, 650,450);
  util_add_route(this.routes, 50,450);
  util_add_route(this.routes, 50, 50);*/
}

World.prototype = {
  // helper function to get closest colliding walls/agents
  stuff_collide_: function(id, p1, p2, check_walls, check_agents) {
    var minres = false;

    // collide with walls
    if(check_walls) {
      for(var i=0,n=this.walls.length;i<n;i++) {
        var wall = this.walls[i];
        var res = line_intersect(p1, p2, wall.p1, wall.p2);
        if(res) {
          res.type = 0; // 0 is wall
          if(!minres) { minres=res; }
          else {
            if(res.ua < minres.ua) { minres = res; }
          }
        }
      }
    }

    if (check_agents){
      for(var i=0,n=this.agents.length;i<n;i++) {
        var it = this.agents[i];
        if (it.id === id) continue;
        var res = line_point_intersect(p1, p2, it.p, 2*it.rad);
        if(res) {
          res.type = 1; // store type of item
          if(!minres) { minres=res; }
          else{
            if(res.ua < minres.ua) { minres = res; }
          }
        }
      }
    }
    return minres;
  },
  tick: function() {
    // tick the environment
    this.clock++;

    // fix input to all agents based on environment
    // process eyes
    this.collpoints = [];
    for(var i=0,n=this.agents.length;i<n;i++) {
      var a = this.agents[i];
      for(var ei=0,ne=a.eyes.length;ei<ne;ei++) {
        var e = a.eyes[ei];
        // we have a line from p to p->eyep
        var eyep = new Vec(a.p.x + e.max_range * Math.sin(a.angle + e.angle),
                           a.p.y + e.max_range * Math.cos(a.angle + e.angle));
        var res = this.stuff_collide_(a.id, a.p, eyep, true, true);
        if(res) {
          // eye collided with wall
          e.sensed_proximity = res.up.dist_from(a.p);
          e.sensed_type = res.type;
        } else {
          e.sensed_proximity = e.max_range;
          e.sensed_type = -1;
        }
      }
    }

    // sensing the angle with reference line
    for(var i=0,n=this.agents.length;i<n;i++) {
      var a = this.agents[i];
      var min_dist = Number.MAX_VALUE, argmin = -1;
      var v1, v2;
      for (var j=0; j<w.routes.length-1; j++){
        v1=w.routes[j], v2=w.routes[j+1];
        var dist = a.p.dist_from_seg(v1,v2);
        if (dist < min_dist){
          min_dist = dist;
          argmin = j;
        }
      }
      v1=this.routes[argmin], v2=this.routes[argmin+1];
      var refline = new Vec(v2.x-v1.x, v2.y-v1.y);
      var angline = new Vec(Math.cos(a.angle), Math.sin(a.angle));
      a.ref_angle = refline.angle(angline);
    }

    // let the agents behave in the world based on their input
    for(var i=0,n=this.agents.length;i<n;i++) {
      this.agents[i].forward();
    }

    // apply outputs of agents on evironment
    for(var i=0,n=this.agents.length;i<n;i++) {
      var a = this.agents[i];
      a.op = a.p; // back up old position
      a.oangle = a.angle; // and angle

      // steer the agent according to outputs of wheel velocities
      var v = new Vec(0, a.rad / 2.0);
      v = v.rotate(a.angle + Math.PI/2);
      var w1p = a.p.add(v); // positions of wheel 1 and 2
      var w2p = a.p.sub(v);
      var vv = a.p.sub(w2p);
      vv = vv.rotate(-a.rot1);
      var vv2 = a.p.sub(w1p);
      vv2 = vv2.rotate(a.rot2);
      var np = w2p.add(vv);
      np.scale(0.5);
      var np2 = w1p.add(vv2);
      np2.scale(0.5);
      a.p = np.add(np2);

      a.angle -= a.rot1;
      if(a.angle<0)a.angle+=2*Math.PI;
      a.angle += a.rot2;
      if(a.angle>2*Math.PI)a.angle-=2*Math.PI;

      // agent is trying to move from p to op. Check walls
      var res = this.stuff_collide_(a.id, a.op, a.p, true, true);
      if(res) { // wall collision! reset position
        a.p = a.op;
        a.collide_signal = 1;
      }

      // handle boundary conditions
      if(a.p.x<0)a.p.x=0;
      if(a.p.x>this.W)a.p.x=this.W;
      if(a.p.y<0)a.p.y=0;
      if(a.p.y>this.H)a.p.y=this.H;
    }

    // compute travel distance signal
    for(var i=0; i<this.agents.length; i++){
      var a = this.agents[i];
      var min_dist = Number.MAX_VALUE;
      var argmin = -1;
      var v1, v2;
      for (var j=0; j<this.routes.length-1; j++){
        v1=this.routes[j], v2=this.routes[j+1];
        var dist = a.p.dist_from_seg(v1,v2);
        if (dist < min_dist){
          min_dist = dist;
          argmin = j;
        }
      }
      // deviation signal
      //a.deviation_signal += min_dist/40;
      // travel signal
      var disp = a.p.sub(a.op);
      v1=this.routes[argmin], v2=this.routes[argmin+1];
      var refline = new Vec(v2.x-v1.x, v2.y-v1.y);
      a.travel_signal = disp.project(refline)/(a.rad/2.0);
    }

    // agents are given the opportunity to learn based on feedback of their action on environment
    for(var i=0,n=this.agents.length;i<n;i++) {
      this.agents[i].backward();
    }
  }
}

// Eye sensor has a maximum range and senses walls
var Eye = function(angle) {
  this.angle = angle; // angle relative to agent its on
  this.max_range = 85;
  this.sensed_proximity = 85; // what the eye is seeing. will be set in world.tick()
  this.sensed_type = -1; // what does the eye see?
}

// A single agent
var Agent = function(id) {
  this.id = id;
  // positional information
  this.p = new Vec(50, 50);
  this.op = this.p; // old position
  this.angle = 0; // direction facing
  this.ref_angle = 0;

  this.actions = [];
  this.actions.push([1,1]);
  this.actions.push([0.8,1]);
  this.actions.push([1,0.8]);
  this.actions.push([0.5,0]);
  this.actions.push([0,0.5]);

  // properties
  this.rad = 10;
  this.eyes = [];
  for(var k=0;k<9;k++) { this.eyes.push(new Eye((k-3)*0.25)); }

  //this.brain = new deepqlearn.Brain(this.eyes.length+1, this.actions.length);
  var spec = document.getElementById('qspec').value;
  eval(spec);
  this.brain = brain;

  this.reward_bonus = 0.0;
  this.travel_signal = 0.0;
  //this.deviation_signal = 0.0;

  // outputs on world
  this.rot1 = 0.0; // rotation speed of 1st wheel
  this.rot2 = 0.0; // rotation speed of 2nd wheel

  this.prevactionix = -1;
}
Agent.prototype = {
  forward: function() {
    // in forward pass the agent simply behaves in the environment
    // create input to brain
    var num_eyes = this.eyes.length;
    //var input_array = new Array(num_eyes * 3 + 1);
    var input_array = new Array(num_eyes*2 + 1);
    for(var i=0;i<num_eyes;i++) {
      var e = this.eyes[i];
      //input_array[i*3] = 1.0;
      //input_array[i*3+1] = 1.0;
      //input_array[i*3+2] = 1.0;
      input_array[i*2] = 1.0;
      input_array[i*2+1] = 1.0;
      if(e.sensed_type !== -1) {
        // sensed_type is 0 for wall, 1 for food and 2 for poison.
        // lets do a 1-of-k encoding into the input array
        //input_array[i*3 + e.sensed_type] = e.sensed_proximity/e.max_range; // normalize to [0,1]
        input_array[i*2+e.sensed_type] = e.sensed_proximity/e.max_range;
      }
    }
    input_array[num_eyes*2] = 1.0-(this.ref_angle/Math.PI);

    // get action from brain
    var actionix = this.brain.forward(input_array);
    var action = this.actions[actionix];
    this.actionix = actionix; //back this up

    // demultiplex into behavior variables
    this.rot1 = action[0]*1;
    this.rot2 = action[1]*1;
  },
  backward: function() {
    // in backward pass agent learns.
    // compute reward
    var proximity_reward = 0.0;
    var num_eyes = this.eyes.length;
    for(var i=0;i<num_eyes;i++) {
      var e = this.eyes[i];
      // agents dont like to see walls, especially up close
      proximity_reward += e.sensed_proximity/e.max_range;
      //proximity_reward += e.sensed_type === 0 ? e.sensed_proximity/e.max_range : 1.0;
    }
    proximity_reward = proximity_reward/num_eyes;
    proximity_reward = Math.min(1.0, proximity_reward * 2);

    // really penalize collision
    //proximity_reward -= (this.collide_signal === 1) ? 0.5: 0;
    //this.collide_signal = 0;

    // agents like to go straight forward
    //var forward_reward = 0.0;
    //if(this.actionix === 0 && proximity_reward > 0.75)
    //  forward_reward = 0.1 * proximity_reward;

    // agents like to follow the path
    //var travel_reward = this.travel_signal;
    var travel_reward;
    if (this.travel_signal < 0)
      travel_reward = Math.min(-2.0, this.travel_signal * 3);
    else
      travel_reward = this.travel_signal * 2;
    this.travel_signal = 0.0;

    var reward = proximity_reward + travel_reward;
    //console.log(proximity_reward, travel_reward);

    // pass to brain for learning
    this.brain.backward(reward);
  }
}

function draw_net() {
  if(simspeed <=1) {
    // we will always draw at these speeds
  } else {
    if(w.clock % 50 !== 0) return;  // do this sparingly
  }

  var canvas = document.getElementById("net_canvas");
  var ctx = canvas.getContext("2d");
  var W = canvas.width;
  var H = canvas.height;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  var L = w.agents[0].brain.value_net.layers;
  var dx = (W - 50)/L.length;
  var x = 10;
  var y = 40;
  ctx.font="12px Verdana";
  ctx.fillStyle = "rgb(0,0,0)";
  ctx.fillText("Value Function Approximating Neural Network:", 10, 14);
  for(var k=0;k<L.length;k++) {
    if(typeof(L[k].out_act)==='undefined') continue; // maybe not yet ready
    var kw = L[k].out_act.w;
    var n = kw.length;
    var dy = (H-50)/n;
    ctx.fillStyle = "rgb(0,0,0)";
    ctx.fillText(L[k].layer_type + "(" + n + ")", x, 35);
    for(var q=0;q<n;q++) {
      var v = Math.floor(kw[q]*100);
      if(v >= 0) ctx.fillStyle = "rgb(0,0," + v + ")";
      if(v < 0) ctx.fillStyle = "rgb(" + (-v) + ",0,0)";
      ctx.fillRect(x,y,10,10);
      y += 12;
      if(y>H-25) { y = 40; x += 12};
    }
    x += 50;
    y = 40;
  }
}

var reward_graph = new cnnvis.Graph();
function draw_stats() {
  var canvas = document.getElementById("vis_canvas");
  var ctx = canvas.getContext("2d");
  var W = canvas.width;
  var H = canvas.height;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  var a = w.agents[0];
  var b = a.brain;
  var netin = b.last_input_array;
  ctx.strokeStyle = "rgb(0,0,0)";
  //ctx.font="12px Verdana";
  //ctx.fillText("Current state:",10,10);
  ctx.lineWidth = 10;
  ctx.beginPath();
  for(var k=0,n=netin.length;k<n;k++) {
    ctx.moveTo(10+k*12, 120);
    ctx.lineTo(10+k*12, 120 - netin[k] * 100);
  }
  ctx.stroke();

  if(w.clock % 200 === 0) {
    reward_graph.add(w.clock/200, b.average_reward_window.get_average());
    var gcanvas = document.getElementById("graph_canvas");
    reward_graph.drawSelf(gcanvas);
  }
}

// Draw everything
function draw() {
  ctx.lineWidth = 2;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  var agents = w.agents;

  // draw walls in environment
  ctx.strokeStyle = "rgb(0,0,0)";
  ctx.beginPath();
  for(var i=0,n=w.walls.length;i<n;i++) {
    var q = w.walls[i];
    ctx.moveTo(q.p1.x, q.p1.y);
    ctx.lineTo(q.p2.x, q.p2.y);
  }
  ctx.stroke();

  // draw routes in environment
  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgb(128,128,128)";
  ctx.beginPath();
  var headlen = 10;
  for(var i=0; i<w.routes.length-1; i++){
    var x1 = w.routes[i].x, y1 = w.routes[i].y;
    var x2 = w.routes[i+1].x, y2 = w.routes[i+1].y;
    var xm = (x1+x2)/2, ym = (y1+y2)/2;
    var angle = Math.atan2(y2-y1, x2-x1);
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.moveTo(xm, ym);
    ctx.lineTo(xm-headlen*Math.cos(angle-Math.PI/6),
               ym-headlen*Math.sin(angle-Math.PI/6));
    ctx.moveTo(xm, ym);
    ctx.lineTo(xm-headlen*Math.cos(angle+Math.PI/6),
               ym-headlen*Math.sin(angle+Math.PI/6));
  }
  ctx.stroke();

  // draw agents
  // color agent based on reward it is experiencing at the moment
  var r = Math.floor(agents[0].brain.latest_reward/2*255);
  if(r>255)r=255;if(r<0)r=0;
  ctx.fillStyle = "rgb(" + r + ", 128, 128)";
  ctx.strokeStyle = "rgb(0,0,0)";
  for(var i=0,n=agents.length;i<n;i++) {
    var a = agents[i];

    // draw agents body
    ctx.beginPath();
    ctx.arc(a.op.x, a.op.y, a.rad, 0, Math.PI*2, true);
    ctx.fill();
    ctx.stroke();

    // draw agents sight
    for(var ei=0,ne=a.eyes.length;ei<ne;ei++) {
      var e = a.eyes[ei];
      var sr = e.sensed_proximity;
      if(e.sensed_type === -1 || e.sensed_type === 0) {
        ctx.strokeStyle = "rgb(0,0,0)"; // wall or nothing
      }
      if(e.sensed_type === 1) { ctx.strokeStyle = "rgb(255,150,150)"; } // apples
      if(e.sensed_type === 2) { ctx.strokeStyle = "rgb(150,255,150)"; } // poison
      ctx.beginPath();
      ctx.moveTo(a.op.x, a.op.y);
      ctx.lineTo(a.op.x + sr * Math.sin(a.oangle + e.angle),
                 a.op.y + sr * Math.cos(a.oangle + e.angle));
      ctx.stroke();
    }
  }

  w.agents[0].brain.visSelf(document.getElementById('brain_info_div'));
}

// Tick the world
function tick() {
  w.tick();
  if(!skipdraw || w.clock % 50 === 0) {
    draw();
    draw_stats();
    draw_net();
  }
}

var simspeed = 2;
function goveryfast() {
  window.clearInterval(current_interval_id);
  current_interval_id = setInterval(tick, 0);
  skipdraw = true;
  simspeed = 3;
}
function gofast() {
  window.clearInterval(current_interval_id);
  current_interval_id = setInterval(tick, 0);
  skipdraw = false;
  simspeed = 2;
}
function gonormal() {
  window.clearInterval(current_interval_id);
  current_interval_id = setInterval(tick, 30);
  skipdraw = false;
  simspeed = 1;
}
function goslow() {
  window.clearInterval(current_interval_id);
  current_interval_id = setInterval(tick, 200);
  skipdraw = false;
  simspeed = 0;
}

function savenet() {
  var j = w.agents[0].brain.value_net.toJSON();
  var t = JSON.stringify(j);
  document.getElementById('tt').value = t;
}

function loadnet() {
  var t = document.getElementById('tt').value;
  var j = JSON.parse(t);
  w.agents[0].brain.value_net.fromJSON(j);
  stoplearn(); // also stop learning
  gonormal();
}

function startlearn() {
  w.agents[0].brain.learning = true;
}
function stoplearn() {
  w.agents[0].brain.learning = false;
}

function reload() {
  w.agents = [new Agent()]; // this should simply work. I think... ;\
  reward_graph = new cnnvis.Graph(); // reinit
}

var w; // global world object
var current_interval_id;
var skipdraw = false;
function start() {
  canvas = document.getElementById("canvas");
  ctx = canvas.getContext("2d");

  w = new World();
  num_agents = 1;
  for (var i=0; i<num_agents; i++)
    w.agents.push(new Agent(i));

  gofast();
}
