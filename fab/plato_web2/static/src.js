// Three.js extension to allow custom matrices without baking them in
THREE.Object3D.prototype._updateMatrix = THREE.Object3D.prototype.updateMatrix;
THREE.Object3D.prototype.updateMatrix = function() {
  this._updateMatrix();
  if (this.customMatrix != null)
    this.matrix.multiply(this.customMatrix);
};

THREE.BufferGeometry.prototype.computeBoundingBoxCustom = function () {
  if (!this.__platoMatrix) {
    this.computeBoundingBox();
    return;
  }

  if ( this.boundingBox === null ) {
    this.boundingBox = new THREE.Box3();
  }

  if (this.attributes[ 'position' ].array) {
    // Copy values out (this is supposed to copy, but double check!);
    var positions = new Float32Array(this.attributes[ 'position' ].array);
    this.__platoMatrix.applyToVector3Array(positions);

    var bb = this.boundingBox;

    if ( positions.length >= 3 ) {
      bb.min.x = bb.max.x = positions[ 0 ];
      bb.min.y = bb.max.y = positions[ 1 ];
      bb.min.z = bb.max.z = positions[ 2 ];
    }

    for ( var i = 3, il = positions.length; i < il; i += 3 ) {
      var x = positions[ i ];
      var y = positions[ i + 1 ];
      var z = positions[ i + 2 ];

      // bounding box
      if ( x < bb.min.x ) {
	bb.min.x = x;
      } else if ( x > bb.max.x ) {
	bb.max.x = x;
      }
      if ( y < bb.min.y ) {
	bb.min.y = y;
      } else if ( y > bb.max.y ) {
	bb.max.y = y;
      }
      if ( z < bb.min.z ) {
	bb.min.z = z;
      } else if ( z > bb.max.z ) {
	bb.max.z = z;
      }
    }
  } else {
    this.boundingBox.min.set( 0, 0, 0 );
    this.boundingBox.max.set( 0, 0, 0 );
  }

  if ( isNaN( this.boundingBox.min.x ) || isNaN( this.boundingBox.min.y ) || isNaN( this.boundingBox.min.z ) ) {
    console.error( 'THREE.BufferGeometry.computeBoundingBox: Computed min/max have NaN values. The "position" attribute is likely to have NaN values.' );
  }
};


function timed_log(message) {
  var d = new Date();
  console.log(d.getMinutes() + ":" + d.getSeconds() + ":" + d.getMilliseconds() + "  " + message);
}

function expandCollapse(this_id) {
  if ($("#" + this_id).text() === "+") {
    $("#" + this_id + "-cont").show();
    $("#" + this_id).text("-");
  } else {
    $("#" + this_id + "-cont").hide();
    $("#" + this_id).text("+");
  }
}

function getKeys(dict) {
  var keys = [];
  for (var key in dict) {
    if (dict.hasOwnProperty(key)) {
      keys.push(key);
    }
  }
  return keys;
}

// Test -------------------------------------------------------------------
var mesh;
var mesh_info = null;
var mesh_binary = null;
var start = new Date();

function FillFromList(buf, list) {
  for (var i = 0; i < buf.length; ++i) {
    buf[i] = list[i];
  }
}

// function BufferedGeometryFromBinary_old(binary_data) {
//   timed_log("Parsing binary data");
//   var meta = new Int32Array(binary_data, 0, 2);
//   var n_vertices = meta[0];
//   var n_triangles = meta[1];
//   timed_log("Wtf?");
//   var geometry = new THREE.BufferGeometry();
//   if (n_vertices == 0) {
//     timed_log("Using unindexed geometry");
//     var positions = new Float32Array(binary_data, 2 * 4, n_triangles * 3 * 3);
//     geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
//   } else {
//     timed_log("Using indexed geometry");
//     var positions = new Float32Array(binary_data, 2 * 4, n_vertices * 3);
//     var triangles = new Uint16Array(binary_data, 2 * 4 + n_vertices * 3 * 4, n_triangles * 3);
//     geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
//     geometry.addAttribute( 'index', new THREE.BufferAttribute( triangles, 1 ) );
//   }

//   var chunkSize = 65536;
//   var offsets = n_triangles / chunkSize;
//   for ( var i = 0; i < offsets; i ++ ) {
//     var offset = {
//       start: i * chunkSize * 3,
//       index: i * chunkSize * 3,
//       count: Math.min( n_triangles - ( i * chunkSize ), chunkSize ) * 3
//     };
//     geometry.offsets.push( offset );
//   }
//   timed_log("Done parsing binary data");
// //  geometry.computeBoundingSphere();
//   timed_log("Done computing sphere");

//   return geometry;
// }

function MeshesToObjString(meshes) {
  var v_lines = [];
  var f_lines = [];
  var start_index = 1;
  for (var i = 0; i < meshes.length; ++i) {
    var mesh = meshes[i];
    var position_array = mesh.geometry.attributes.position.array;
    var index_array = [];
    if (mesh.geometry.attributes.index) {
      index_array = mesh.geometry.attributes.index.array;
    }

    for (var v = 0; v < position_array.length; v+=3) {
      var vec = new THREE.Vector4(
        position_array[v],
        position_array[v + 1],
        position_array[v + 2],
        1.0);
      vec.applyMatrix4(mesh.matrix);
      v_lines.push("v " +
                   (vec.x / vec.w)  + " " +
                   (vec.y / vec.w) + " " +
                   (vec.z/ vec.w));
    }
    if (index_array.length > 0) {
      console.log("Writing indexed geometry");
      for (var t = 0; t < index_array.length; t+= 3) {
        f_lines.push("f " +
                     (start_index + index_array[t]) + " " +
                     (start_index + index_array[t + 1]) + " " +
                     (start_index + index_array[t + 2]));
      }
    } else {
      console.log("Writing UN-indexed geometry");
      for (var v = 0; v < position_array.length; v+=3) {
        f_lines.push("f " +
                     (start_index + v + 1) + " " +
                     (start_index + v + 2) + " " +
                     (start_index + v + 3));
      }
    }
    start_index += position_array.length / 3;
  }
  return v_lines.concat(f_lines);
}

function UpdateMeshLink() {
  var lines = MeshesToObjString(mesh_view.meshes);

  var str = lines.join("\n");
  var byteNumbers = new Array(str.length);
  for (var i = 0; i < str.length; i++) {
    byteNumbers[i] = str.charCodeAt(i);
  }

  var byteArray = new Uint8Array(byteNumbers);

  var blob = new Blob([byteArray], {type: "text/plain"});
  var blobUrl = URL.createObjectURL(blob);

  var file = controller.encodeValues() + ".obj";
  $("#obj").text(file);
  var link = $("#obj").get(0);
  link.download = file;

  link.href = blobUrl;

//"text/plain;charset=utf-8,"
//'data:application/json;base64,'
//    + window.btoa(
  //    unescape(
//    +    encodeURIComponent(lines.join("\n"));

//                            JSON.stringify(controller.results)));
}

function BufferedGeometriesFromBinary(binary_data) {
  timed_log("Parsing binary data");
  var n_meshes = new Int32Array(binary_data, 0, 1);
  console.log("Decoding " + n_meshes[0] + " meshes");

  var geometries = [];
  var read_start = 4;  // 4 bytes used for n_meshes read above
  for (var m = 0; m < n_meshes[0]; ++m) {
    var meta = new Int32Array(binary_data, read_start, 2);
    read_start += 2 * 4;
    var n_vertices = meta[0];
    var n_triangles = meta[1];
    timed_log(n_vertices + " verts, " + n_triangles + "triangles");

    var m_arr = new Float32Array(binary_data, read_start, 16);
    read_start += 16 * 4;
    var matrix = new THREE.Matrix4();
    matrix.set(m_arr[0], m_arr[1], m_arr[2], m_arr[3],
               m_arr[4], m_arr[5], m_arr[6], m_arr[7],
               m_arr[8], m_arr[9], m_arr[10], m_arr[11],
               m_arr[12], m_arr[13], m_arr[14], m_arr[15]);

    var geometry = new THREE.BufferGeometry();
    geometry.__platoMatrix = matrix;
    if (n_vertices == 0) {
      timed_log("Using unindexed geometry");
      var positions = new Float32Array(binary_data, read_start, n_triangles * 3 * 3);
      read_start += n_triangles * 3 * 3 * 4;  // 4 bytes * 3/ vert * 3 vert/ tri
      geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
    } else {
      timed_log("Using indexed geometry");
      var positions = new Float32Array(binary_data, read_start, n_vertices * 3);
      read_start += n_vertices * 3 * 4;
      var triangles = new Uint16Array(binary_data, read_start, n_triangles * 3);
      read_start += n_triangles * 3 * 2;
      geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );
      geometry.addAttribute( 'index', new THREE.BufferAttribute( triangles, 1 ) );
    }

    var chunkSize = 65536;
    var offsets = 1; //n_triangles / chunkSize;
    for ( var i = 0; i < offsets; i ++ ) {
      var offset = {
        start: i * chunkSize * 3,
        index: i * chunkSize * 3,
        count: n_triangles * 3 //Math.min( n_triangles - ( i * chunkSize ), chunkSize ) * 3
      };
      geometry.offsets.push( offset );
    }
    timed_log("Done parsing mesh " + m);
    geometries.push(geometry);
  }
  return geometries;
}


// http://benaadams.github.io/three.js/examples/webgl_interleavedbuffergeometry.html
function CreateBufferedGeometry2() {
  var triangles = 2;
  var geometry = new THREE.BufferGeometry();


  var positions = new Float32Array( 3 * 3 * 3 );
  FillFromList(positions, [0.0, 0.0, 1.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 0.0,
                           1.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 0.0,
                           1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0 ]);
  geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );

  var chunkSize = 21845;
  var offsets = triangles / chunkSize;
  for ( var i = 0; i < offsets; i ++ ) {
    var offset = {
      start: i * chunkSize * 3,
      index: i * chunkSize * 3,
      count: Math.min( triangles - ( i * chunkSize ), chunkSize ) * 3
    };
    geometry.offsets.push( offset );
  }

  geometry.computeBoundingSphere();

  return geometry;
}


// http://benaadams.github.io/three.js/examples/webgl_interleavedbuffergeometry.html
function CreateBufferedGeometry() {
  var triangles = 2;
  var geometry = new THREE.BufferGeometry();

  // break geometry into
  // chunks of 21,845 triangles (3 unique vertices per triangle)
  // for indices to fit into 16 bit integer number
  // floor(2^16 / 3) = 21845

  var chunkSize = 21845;

  var indices = new Uint16Array( triangles * 3 );
  FillFromList(indices, [0, 1, 2, 2, 1, 3]);

  // indices[ i ] = i % ( 3 * chunkSize );

  var positions = new Float32Array( 4 * 3 );
  FillFromList(positions, [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);

  geometry.addAttribute( 'index', new THREE.BufferAttribute( indices, 1 ) );
  geometry.addAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );

  var offsets = triangles / chunkSize;
  for ( var i = 0; i < offsets; i ++ ) {
    var offset = {
      start: i * chunkSize * 3,
      index: i * chunkSize * 3,
      count: Math.min( triangles - ( i * chunkSize ), chunkSize ) * 3
    };
    geometry.offsets.push( offset );
  }

  geometry.computeBoundingSphere();

  return geometry;
}

function ArrayBufferFromList(list, is_float) {
  var res = null;
  if (is_float) {
    res = new Float32Array(list.length);
  } else {
    res = new Uint16Array(list.length);
  }
  for (var i = 0; i < list.length; ++i) {
    res[i] = list[i];
  }
  return res;
};




function on_mesh(jmesh) {
  timed_log("Processing mesh ( " + (new Date() - start) + " ms from JS load");
  var d = new Date();
  var loader = new THREE.JSONLoader();
  mesh = loader.parse(jmesh, null);
  console.log("Parsing took: " + (new Date() - d) + " ms");
  mesh_view.setGeometry(CreateBufferedGeometry(), RenderUtils.shaderMaterial("red"));//mesh.geometry);
}



// Control point control -------------------------------------------------------

// prefix is something like "midpt", and will be automatically appended
// with "_x" and "_y"
function ControlPtControl(control_name_prefix) {
  this.handle = $("#pt-handle");
  this.handle.show();

  this.name_x = control_name_prefix + "_x";
  this.name_y = control_name_prefix + "_y";
  this.setValuesFromSliders();

  // Slider must move when the handle
  this.handle.mousedown(function(c) {
    return function(e) {c.handleMousedown();}; }(this));
}

ControlPtControl.prototype.destruct = function() {
  $('#preview-window').off();
  this.handle.off();
  this.handle.hide();
}

ControlPtControl.prototype.handleMousedown = function(e) {
  $('#preview-window').mousemove(function(c) {
    return function(e) { c.handleMousemove(e); }; }(this));

  $('#preview-window').mouseup(
    function(e) { $(this).off(); });
}

ControlPtControl.prototype.setValuesFromSliders = function() {
  this.value_x = controller.ui_controls[this.name_x].slider_elem.slider("value");
  this.value_y = controller.ui_controls[this.name_y].slider_elem.slider("value");

  var screen = mesh_view.getScreenCoords(
    new THREE.Vector3(this.value_x, this.value_y, 0));
  this.handle.css("left", screen.x - 15);
  this.handle.css("top", screen.y - 15);
}

ControlPtControl.prototype.handleMousemove = function(e) {
  var parentOffset = $(this.handle).parent().offset();
  var relX = e.pageX - parentOffset.left;
  var relY = e.pageY - parentOffset.top;

  if (!mesh_view.isValidInGrid(relX, relY)) {
    return false;
  }

  this.handle.css("left", relX - 15);
  this.handle.css("top", relY - 15);

  // Compute actual positions
  var vals = mesh_view.getWorldCoords(relX, relY);
  this.value_x = vals.x;
  this.value_y = vals.y;

  controller.ui_controls[this.name_x].slider_elem.slider("value", this.value_x);
  controller.ui_controls[this.name_y].slider_elem.slider("value", this.value_y);
  controller.sendMessage("SEED " + controller.encodeValues());
}


// Slider ----------------------------------------------------------------------

function SliderControl(parent, callback, control) {
  this.control = control;
  this.invalid_intervals = [];

  var my_dom = $('<div class="control" id="ctr-' + control.name + '">' +
                 '<div class="name"> jaggedness </div>' +
                 '<div class="slider-outer">' +
                 '<div class="min">0.1</div>' +
                 '<div class="slider-inner">' +
                 '<canvas class="tickmarks" width="300" height="100"></canvas>' +
                 '<div class="slider">' +
                 '<div class="curval">5.16</div>' +
                 '</div>' +
                 '</div>' +
                 '<div class="max">55.3</div>' +
                 '</div>' +
                 '</div>');

  this.is_double = (control.type === "TYPE_DOUBLE");

  if (this.is_double) {
    this.formatValue = function(v) {
      return v.toFixed(1);
    };
  } else {
    this.formatValue = function(v) { return v; };
  }

  this.slider_elem = my_dom.find(".slider");
  this.cur_val = my_dom.find(".curval");
  this.cur_val.text(this.formatValue(control.value));
  my_dom.find(".name").text( control.name );
  my_dom.find(".min").text(this.formatValue(control.range[0]));
  my_dom.find(".max").text(this.formatValue(control.range[1]));

  var step = 1;
  if (this.is_double) {
    step = (control.range[1] - control.range[0]) / 32.0;
  }
  var n_steps = (control.range[1] - control.range[0]) / step + 1;
  this.step = step;

  my_dom.appendTo(parent);

  this.slider_elem.slider({ min: control.range[0],
                            max: control.range[1],
                            step: step,
                            value: control.value,
                            change: function(c) {
                              return function(event, ui) {
                                c.setValueLabel(ui.value);
                              };
                            }(this),
                            stop: function(c) {
                              return function(event, ui) {
                                // TODO: update slider values
                                if (stats) { stats.slider_clicks++; }

                                };
                            }(this),
                            slide: function(c) {
                              return function(event, ui) {
                                // TODO: hacky! change
                                var delta = Math.abs(parseFloat(c.cur_val.text()) - ui.value);

                                if (c.notValid(ui.value)) {
                                  return false;
                                }
                                controller.handleValidSliderChange(c.control.name, ui.value);
                              };
                            }(this)
                          });
  this.cur_val.css("padding-left",
                   $(".ui-slider-handle", this.slider_elem).css("left"));

  // Fill int tickmarks
  var canvas = my_dom.find(".tickmarks");
  var ctx = canvas[0].getContext('2d');
  ctx.fillStyle = 'rgb(170,170,170)';
  ctx.lineWidth = 1;

  var width = 300; // pixels
  var height = 100; //pixels
  n_steps = n_steps - 1;
  for (var i = 0; i < n_steps; ++i) {
    ctx.beginPath();
//    console.log(i * width / n_steps);
    ctx.moveTo(i * width / n_steps, 0);
    ctx.lineTo(i * width / n_steps, height);
    ctx.stroke();
  }
}

SliderControl.prototype.notValid = function(val) {
  for (var i = 0; i < this.invalid_intervals.length; ++i) {
    if (val >= this.invalid_intervals[i][0] &&
        val <= this.invalid_intervals[i][1]) {
      return true;
    }
  }
  return false;
}

SliderControl.prototype.setValueLabel = function(val) {
  this.cur_val.text(this.formatValue(val));
  this.cur_val.css("padding-left",
                $(".ui-slider-handle", this.slider_elem).css("left"));
}

SliderControl.prototype.approxEqual = function(a, b) {
  return Math.abs(a-b) < this.step;
}

SliderControl.prototype.convertToInvalidIntervals = function(intervals) {
  var invalid = [];

  if (intervals.length == 0) {
    invalid.push([this.control.range[0], this.control.range[1]]);
    return invalid;
  }

  if (this.is_double) {
    if (!this.approxEqual(intervals[0][0], this.control.range[0])) {
      invalid.push([this.control.range[0], intervals[0][0]]);
    }

    for (var i = 0; i < intervals.length - 1; ++i) {
      invalid.push([intervals[i][1], intervals[i+1][0]]);
    }

    if (!this.approxEqual(intervals[intervals.length - 1][1],
                          this.control.range[1])) {
      invalid.push([intervals[intervals.length - 1][1],
                    this.control.range[1]]);
    }
  } else {
    if (!this.approxEqual(intervals[0][0], this.control.range[0])) {
      invalid.push([this.control.range[0], intervals[0][0] - 0.5]);
    }

    for (var i = 0; i < intervals.length - 1; ++i) {
      invalid.push([intervals[i][1] + 0.5, intervals[i+1][0] - 0.5]);
    }

    if (!this.approxEqual(intervals[intervals.length - 1][1],
                          this.control.range[1])) {
      invalid.push([intervals[intervals.length - 1][1] + 0.5,
                    this.control.range[1]]);
    }
  }

  return invalid;
}

SliderControl.prototype.setValidRanges = function(intervals) {
//  console.log("Handling intervals");
//  console.log(intervals);

  var invalidIntervals = this.convertToInvalidIntervals(intervals);
  this.invalid_intervals = invalidIntervals;

  $(this.slider_elem).find(".range").remove();

  for (var i = 0; i < invalidIntervals.length; ++i) {
    var interval = invalidIntervals[i];
    var length = interval[1] - interval[0];
    var full_range = this.control.range[1] - this.control.range[0];

    var range = $('<div class="range"></div>');
    range.width( length / full_range * 100 + "%");
    range.css({top: "0", left : (interval[0] - this.control.range[0]) / full_range * 100 + "%"});
    range.appendTo(this.slider_elem);
  }
}

// Grid wrapper ----------------------------------------------------------------

// Wraps:
// {x_dim:2,y_dim:2,pos: [0.42,2,0.7,2,0.42,1.701,0.7,1.701],codes: [0,0,0,2]
function GridWrap(raw_grid) {
  this.raw = raw_grid;
  this.x_dim = this.raw.x_dim;
  this.y_dim = this.raw.y_dim;
}

GridWrap.prototype.getVector3 = function(i, j) {
  var index = this.getIndex(i, j);
  var x = this.raw.pos[index * 2];
  var y = this.raw.pos[index * 2 + 1];
  return new THREE.Vector3(x, y, 0);
}

GridWrap.prototype.getCode = function(i, j) {
  var index = this.getIndex(i, j);
  return this.raw.codes[index];
}

GridWrap.prototype.getIndex = function(i, j) {
  return j * this.raw.x_dim + i;
}

// Stats -----------------------------------------------------------------------
function TaskStats() {
  this.newTask();
}

TaskStats.prototype.init = function(elem) {
  this.out_elem = $('#taskinfo', elem);

  $('#taskstart', elem).button().click(
    function(ts) {
      return function() {ts.newTask();};
    }(this));

  $('#taskend', elem).button().click(
    function(ts) {
      return function() {
        ts.endTask();
        ts.showStats();
      };
    }(this));

  // $('.seed-cont').click(
  //   function(ts) {
  //     return function() {
  //       ts.seed_clicks = ts.seed_clicks + 1;
  //     };
  //   }(this));
}

// DURATION_S,SEED_CLICKS,SLIDER_CLICKS,VIEWPOINT_CLICKS,FINAL_PT
TaskStats.prototype.showStats = function() {
  var formatted = ((this.task_end - this.task_start) / 1000) + "," +
    this.seed_clicks + "," + this.slider_clicks + "," + this.viewer_clicks
    + "," + controller.encodeValues();
  this.out_elem.text(formatted);
}

TaskStats.prototype.newTask = function() {
  console.log("started task");
  this.task_start = new Date();
  this.task_end = null;
  this.seed_clicks = 0;
  this.slider_clicks = 0;
  this.viewer_clicks = 0;
}

TaskStats.prototype.incr_viewer_stats = function() {
  if (!this.last_viewer_update) {
    this.last_viewer_update = new Date();
    this.viewer_clicks++;
  }
  var d = new Date();
  if ( (d - this.last_viewer_update) / 1000 >= 3) {
    this.viewer_clicks++;
  }
  this.last_viewer_update = new Date();
}


TaskStats.prototype.endTask = function() {
  this.task_end = new Date();
}

// Controller ------------------------------------------------------------------
function Controller(mv, enable_control_pts) {
  this.enable_control_pts = enable_control_pts;
  this.mesh_view = mv;
  this.info = null;
  this.bounds = null;
  this.ui_controls = {};
  this.control_names = [];

  this.material = "red_plastic";

  this.volume = 0;
  this.weight_elem = null;
  this.cost_elem = null;
}

Controller.prototype.setMaterial = function(name) {
  this.material = name;
  this.mesh_view.setMaterial(RenderUtils.shaderMaterial(this.material));
  this.updateVolumeVars();
}

Controller.prototype.setWeightDisplay = function(elem) {
  this.weight_elem = elem;
}

Controller.prototype.setCostDisplay = function(elem) {
  this.cost_elem = elem;
}

Controller.prototype.updateVolumeVars = function() {
  if (this.volume <= 0) { return; }

  var weight = this.volume * this.material_props[this.material].density;  // grams
  var cost = this.volume * this.material_props[this.material].cost; // dollars

  // Set in the UI
  if (this.weight_elem) {
    this.weight_elem.text(weight.toFixed(2));
  }

  if (this.cost_elem) {
    this.cost_elem.text(cost.toFixed(2));
  }
}

Controller.prototype.init = function() {
  this.ws = new WebSocket("ws://" + window.location.host + window.location.pathname);
  this.ws.binaryType = "arraybuffer";

  this.ws.onopen = function(c){return function() { c.onopen();} }(this);
  this.ws.onmessage = function(c){return function(evt) { c.onmessage(evt);} }(this);
  this.ws.onclose = function(c){return function() { c.onclose();} }(this);
}

Controller.prototype.initMaterials = function(elem) {
  this.material_props = {
    "red_plastic" : { density: 1.0, cost: 1.50 },
    "gold": { density: 19.3, cost: 800.0 },
    "blue_ceramics": { density: 2.5, cost: 0.35 }
  };

  function formatCost(value) {
    return "$" + value.toPrecision(2) + " / cm<sup>3</sup";
  };

  var ddData = [
    {
      text: "Red Plastic",
      value: "red_plastic",
      selected: true,
      description: formatCost(this.material_props["red_plastic"].cost),
      imageSrc: "img/mat_red_plastic.png"
    },
    {
      text: "Blue Ceramics",
      value: "blue_ceramics",
      selected: false,
      description: formatCost(this.material_props["blue_ceramics"].cost),
      imageSrc: "img/mat_blue_ceramics.png"
    },
    {
      text: "Gold",
      value: "gold",
      selected: false,
      description: formatCost(this.material_props["gold"].cost),
      imageSrc: "img/mat_gold.png"
    }
  ];

  $(elem).ddslick({
    data: ddData,
    width: 300,
    imagePosition: "left",
    selectText: "Select material",
    onSelected: function(c) { return function (data) {
      c.setMaterial(data.selectedData.value);
    }; }(this)
  });
}

Controller.prototype.sendMessage = function(str) {
  timed_log("Sending message: " + str);
  this.ws.send(str);
}

Controller.prototype.jumpToSeed = function(seed) {
  if (stats) {
    stats.seed_clicks++;
  }
  this.sendMessage("SEED " + seed);
  var vals = seed.split("_");
  for (var i = 0; i < vals.length; ++i) {
    var num_val = parseFloat(vals[i]);
    console.log("setting val " + i + " to " + num_val);
    this.ui_controls[this.control_names[i]].slider_elem.slider("value", num_val);
  }

  if (this.control_pt) {
    this.control_pt.setValuesFromSliders();
    this.sendMessage("GET-GRID " + this.control_pt.name_x +
                       " " + this.control_pt.name_y);
  }
}

Controller.prototype.encodeValues = function() {
  var res = '';
  for (var i = 0; i < this.control_names.length; ++i) {
    if (i > 0) {
      res = res + "_";
    }
    res = res + this.ui_controls[this.control_names[i]].slider_elem.slider("value");
  }
  return res;
}

Controller.prototype.handleSeeds = function(seeds) {
  var parent = $("#seeds .panel-body");
  for (var i = 0; i < seeds.length; ++i) {
    var my_dom = $('<div class="seed-cont">' +
                   '<img src="/img/' + seeds[i] + '.png"/>' +
                   '<div class="seed-txt">' + seeds[i] + '</div></div>');
    my_dom.appendTo(parent);
    my_dom.click(function(c, s) {
      return function() {
        c.jumpToSeed(s);
        expandCollapse("plusmin1");
      };
    } (this, seeds[i]) );
  }
  $("#seeds").show();
}

Controller.prototype.handleBoundsProps = function(parsed) {
  if ("bounds" in parsed) {
    var keys = getKeys(parsed["bounds"]);
    for (var i = 0; i < keys.length; ++i) {
      var key = keys[i];
      if (key in this.ui_controls) {
        this.ui_controls[key].setValidRanges(parsed["bounds"][key]);
      }
    }
  }

  if ("volume" in parsed) {
    this.volume = parsed["volume"];
    this.updateVolumeVars();
  }
}

Controller.prototype.setControlPtTriggers = function() {
  if (!this.enable_control_pts) {
    return;
  }

  for (var i = 0; i < this.control_names.length; ++i) {
    // HACK
    var name = this.control_names[i];
    if (name == "pt1_x" ||
        name == "pt_x" ||
        name == "midpt_x" ||
        name == "toppt_x") {
      var pt_name = name.substring(0, name.length - 2);
      var my_dom = $(
          '<div class="pt-edit-overlay pt-edit-off" id="' +
          pt_name + '"></div>');
      my_dom.click(function(c, pt_name) {
        return function() {
          if ($(this).hasClass("pt-edit-off")) {
            c.disableControlPt();
            c.enableControlPt(pt_name);
            $(".pt-edit-overlay").removeClass("pt-edit-on");
            $(".pt-edit-overlay").addClass("pt-edit-off");
            $(this).toggleClass("pt-edit-off pt-edit-on");
          } else {
            c.disableControlPt();
            $(".pt-edit-overlay").removeClass("pt-edit-on");
            $(".pt-edit-overlay").addClass("pt-edit-off");
          }
        };
      } (this, pt_name));
      my_dom.css("top", $("#ctr-" + name).offset().top + 5);

      $(".sidebar").scroll(function(md, name) {
        return function() {
          md.css("top", $("#ctr-" + name).offset().top + 5);
        }; }(my_dom, name));

      $("body").append(my_dom);
    }
  }
}

Controller.prototype.processTextMessage = function(m) {
//  console.log(m);
  var parsed = eval('x = ' + m);
  // TODO: JSON.parse is safer
  if (this.info === null && ("controls" in parsed)) {
    this.info = parsed;

    for (var i = 0; i < this.info.controls.length; ++i) {
      var control = this.info.controls[i];
      this.control_names.push(control.name);
      this.ui_controls[control.name] =
        new SliderControl($("#controls"),
                          null,
                          control);
    }
    this.setControlPtTriggers();

    if ("name" in parsed) {
      $("#name").text(parsed["name"]);
    }

    if ("seeds" in parsed) {
      this.handleSeeds(parsed["seeds"]);
    }

    if (this.bounds) {  // if bounds already received
      this.handleBoundsProps(this.bounds);
      this.bounds = null;
    } else if ("bounds" in parsed) {
      this.handleBoundsProps(parsed);
    }

  } else if ("bounds" in parsed) {  // bounds update
    if (this.info === null) {
      this.bounds = parsed;  // if info not yet loaded
    } else {
      this.handleBoundsProps(parsed);
    }
  } else if ("grid" in parsed) {
    console.log("Handling grid");
    this.handleValidGrid(parsed["grid"]);
  }
}

Controller.prototype.handleValidSliderChange = function(name, value) {
  controller.sendMessage(name + " " + value);
  //controller.sendMessage("SEED " + this.encodeValues());

  if (this.control_pt) {
    if (name == this.control_pt.name_x ||
        name == this.control_pt.name_y) {
      this.control_pt.setValuesFromSliders();
    } else {
      this.sendMessage("GET-GRID " + this.control_pt.name_x +
                       " " + this.control_pt.name_y);
    }
  }
}

Controller.prototype.enableControlPt = function(prefix) {
  this.disableControlPt();  // if any previous

  this.grid_enabled = true;
  mesh_view.setOrthoCamera();
  this.control_pt = new ControlPtControl(prefix);
  this.sendMessage("GET-GRID " + this.control_pt.name_x +
                   " " + this.control_pt.name_y);
}

Controller.prototype.disableControlPt = function() {
  if (this.control_pt) {
    this.control_pt.destruct();
    this.control_pt = null;
  }
  this.grid_enabled = false;
  this.grid = null;
  mesh_view.setGrid(null);
  mesh_view.setProjCamera();
}

Controller.prototype.handleValidGrid = function(grid) {
  if (this.grid_enabled) {
    this.grid = new GridWrap(grid);
    mesh_view.setGrid(this.grid);
  }
}

Controller.prototype.onopen = function() {
  timed_log("Connection open");
};

Controller.prototype.onmessage = function (evt) {
  timed_log("Message received");
  //ws_evt = evt;

  if (typeof evt.data === "string") {
    timed_log("Got text message");

    this.processTextMessage(evt.data);

  } else if (evt.data instanceof ArrayBuffer) {
    timed_log("Got arraybuffer message");

    mesh_binary = evt.data;
    var geos = BufferedGeometriesFromBinary(evt.data);
    this.mesh_view.setGeometries(geos, RenderUtils.shaderMaterial(this.material));

    timed_log("Set mesh");
  } else {
    timed_log("Got unknown message");
    console.log(evt);
  }
};

Controller.prototype.onclose = function() {
  // websocket is closed.
  timed_log("Connection closed");
};


// Rendering -------------------------------------------------------------------

function MeshView() {
  this.meshes = null;
  this.model_scale = 1.0;
  this.model_position = new THREE.Vector3();

  // HACK!!! For tess vase, b/c the control pts
  // are relative to the position of the base
  this.control_pt_adjustment = new THREE.Vector3(0.8, 0.2, 0);
}

MeshView.prototype.setMaterial = function(material) {
  if (!this.meshes) {
    return;
  }

  for (var m = 0; m < this.meshes.length; ++m) {
    this.meshes[m].material = material;
  }

  if (this.renderer) {
    this.render();
  }
}

// MeshView.prototype.setGeometry = function(geometry, material) {
//   if (!geometry) {
//     return;
//   }

//   geometry.computeBoundingBox();
//   if (!this.mesh) {
//     this.initScalePosition(geometry.boundingBox);
//   } else {
//     this.scene.remove(this.mesh);
//   }

//   this.mesh = RenderUtils.createMesh(geometry, material);
//   this.mesh.position.set(this.model_position.x,
//                          this.model_position.y,
//                          this.model_position.z);
//   this.mesh.scale.set(this.model_scale, this.model_scale, this.model_scale);
//   this.scene.add(this.mesh);

//   if (this.renderer) {
//     this.render();
//   }
// }

MeshView.prototype.getBoundingBox = function(geometries) {
  var bb = new THREE.Box3();
  if (geometries.length == 0) {
    bb.min.set(-0.5, 0, -0.5);
    bb.max.set(0.5, 1, 0.5);
    return bb;
  }

  geometries[0].computeBoundingBoxCustom();
  bb.min.set(geometries[0].boundingBox.min.x,
             geometries[0].boundingBox.min.y,
             geometries[0].boundingBox.min.z);
  bb.max.set(geometries[0].boundingBox.max.x,
             geometries[0].boundingBox.max.y,
             geometries[0].boundingBox.max.z);
  for (var g = 1; g < geometries.length; ++g) {
    geometries[g].computeBoundingBoxCustom();
    var bb2 = geometries[g].boundingBox;
    for (var i = 0; i < 3; ++i) {
      if (bb2.min[i] < bb.min[i]) {
        bb.min[i] = bb2.min[i];
      }
      if (bb2.max[i] > bb.max[i]) {
        bb.max[i] = bb2.max[i];
      }
    }
  }
  return bb;
}

MeshView.prototype.initScalePosition = function(boundingBox) {
  this.model_scale = RenderUtils.computeGeometryScale(boundingBox);
  this.model_position = new THREE.Vector3(
      -this.model_scale * (boundingBox.max.x + boundingBox.min.x) / 2.0,
      -this.model_scale * boundingBox.min.y + 0.2,
      -this.model_scale * (boundingBox.max.z + boundingBox.min.z) / 2.0);
}

MeshView.prototype.setGeometries = function(geometries, material) {
  if (!this.meshes) {
    var boundingBox = this.getBoundingBox(geometries);
    console.log("Found bounding box");
    console.log(boundingBox);
    this.initScalePosition(boundingBox);
    this.meshes = [];
  } else {
    for (var m = 0; m < this.meshes.length; ++m) {
      this.scene.remove(this.meshes[m]);
    }
    this.meshes = [];
  }

  for (var g = 0; g < geometries.length; ++g) {
    var geometry = geometries[g];
    var mesh = RenderUtils.createMesh(geometry, material);
    mesh.position.set(this.model_position.x,
                      this.model_position.y,
                      this.model_position.z);
    mesh.scale.set(this.model_scale, this.model_scale, this.model_scale);
    if (geometry.__platoMatrix) {
      mesh.customMatrix = geometry.__platoMatrix;
    }
    this.meshes.push(mesh);
    this.scene.add(mesh);
  }

  if (this.renderer) {
    this.render();
  }

  console.log("Rendered");
}

MeshView.prototype.initRendering = function() {
  // Rendering
  this.container = $("#preview-window").get(0);
  var width = this.container.clientWidth;
  var height = this.container.clientHeight;

  $("#valid-overlay").hide();

  this.camera = new THREE.PerspectiveCamera(35, width / height, 1, 15 );
  this.camera.position.set(0, 1.0, 1.3);
  this.cameraTarget = new THREE.Vector3( 0, 0.4, 0);
  this.proj_camera = this.camera;  // safe for later

  this.scene = RenderUtils.createScene();

  this.renderer = RenderUtils.createRenderer();
  this.renderer.setSize(width, height);
  this.container.insertBefore(this.renderer.domElement, this.container.firstChild);
  $("canvas", this.container)[1].width = width;
  $("canvas", this.container)[1].height = height;

  this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
  this.controls.center = this.cameraTarget;
  this.controls.maxDistance = 4;
  this.controls.minDistance = 1.0;

  // fix
  $(window).resize(function(mv){
    return function() { mv.onWindowResize(); };}(this));

  this.startRendering();
}

MeshView.prototype.setGrid = function(grid_wrap) {
  this.grid = grid_wrap;
  this.renderGrid(this.grid);
}

MeshView.prototype.isValidInGrid = function(screenX, screenY) {
  if (!this.ctx || !this.grid) {
    return true;
  }

  var imgd = this.ctx.getImageData(screenX, screenY, 1, 1);
  var pix = imgd.data;
  if (pix[3] > 20) { return true; }
  return false;
}

MeshView.prototype.renderGrid = function(grid_wrap) {
  if (!this.ctx) {
    var canvas = document.getElementById("valid-overlay");
    if (!canvas.getContext) {
      alert("Error! HTML5 canvas not supported. Please try updated Firefox of Chrome.");
    }
    this.ctx = canvas.getContext('2d');
//    this.ctx.fillStyle = 'rgb(70,150,250)';
//    this.ctx.fillRect(20,20,150,100);

    // this.ctx.beginPath();
    // this.ctx.arc(200, 200, 12, 0, 2 * Math.PI, true);
    // this.ctx.fill();
  }
  this.ctx.globalAlpha = 0.5;

  var width = this.container.clientWidth;
  var height = this.container.clientHeight;
  this.ctx.clearRect(0, 0, width, height);
  if (!grid_wrap) {
    $("#valid-overlay").hide();
    return;
  } else {
    $("#valid-overlay").show();
  }

  var valid_color = 'rgb(70,150,250)';
  var invalid_color = 'rgb(50,50,50)';
  for (var i = 0; i < grid_wrap.x_dim - 1; ++i) {
    for (var j = 0; j < grid_wrap.y_dim - 1; ++j) {
      var top_left = 0, top_right = 1, bottom_left = 2, bottom_right = 3;
      var positions = [
        this.getScreenCoords(grid_wrap.getVector3(i, j)),
        this.getScreenCoords(grid_wrap.getVector3(i + 1, j)),
        this.getScreenCoords(grid_wrap.getVector3(i, j + 1)),
        this.getScreenCoords(grid_wrap.getVector3(i + 1, j + 1))];
      var valid = [ grid_wrap.getCode(i, j),
                    grid_wrap.getCode(i + 1, j),
                    grid_wrap.getCode(i, j + 1),
                    grid_wrap.getCode(i + 1, j + 1) ];
      var valid_count = 0;
      for (var v = 0; v < 4; ++v) {
        if (valid[v] == 0) { ++valid_count; }
      }

      if (valid_count == 4) {
        this.ctx.fillStyle = valid_color;
        this.ctx.fillRect(positions[0].x, positions[0].y,
                          positions[1].x - positions[0].x,
                          positions[2].y - positions[0].y);
      }

      // } else if (valid_count == 0 || valid_count == 1) {
      //   this.ctx.fillStyle = invalid_color;
      // } else if (valid_count == 2) {
      //   if (valid[0] == valid[1]) {  // create vertical gradient
      //     var grad = this.ctx.createLinearGradient(
      //       positions[0].x, positions[0].y,
      //       positions[2].x, positions[2].y);
      //     grad.addColorStop(0, valid[0] == 0 ? valid_color : invalid_color);
      //     grad.addColorStop(1, valid[3] == 0 ? valid_color : invalid_color);
      //     this.ctx.fillStyle = grad;
      //   } else if (valid[0] == valid[2]) {
      //     var grad = this.ctx.createLinearGradient(
      //       positions[0].x, positions[0].y,
      //       positions[1].x, positions[1].y);
      //     grad.addColorStop(0, valid[0] == 0 ? valid_color : invalid_color);
      //     grad.addColorStop(1, valid[1] == 0 ? valid_color : invalid_color);
      //     this.ctx.fillStyle = grad;
      //   } else {  // opposite vertices
      //     this.ctx.fillStyle = invalid_color;
      //   }
      // } else {  // 3 valid vertices
      //   var grad = null;
      //   if (valid[0] == 1 || valid[2] == 1) {
      //     grad = this.ctx.createLinearGradient(
      //       positions[0].x, positions[0].y,
      //       positions[2].x, positions[2].y);
      //     grad.addColorStop(0, valid[0] == 0 ? valid_color : invalid_color);
      //     grad.addColorStop(0.5, valid_color);
      //     grad.addColorStop(1, valid[2] == 0 ? valid_color : invalid_color);
      //   } else {
      //     grad = this.ctx.createLinearGradient(
      //       positions[1].x, positions[1].y,
      //       positions[3].x, positions[3].y);
      //     grad.addColorStop(0, valid[1] == 0 ? valid_color : invalid_color);
      //     grad.addColorStop(0.5, valid_color);
      //     grad.addColorStop(1, valid[3] == 0 ? valid_color : invalid_color);
      //   }
      //   this.ctx.fillStyle = grad;
      // }

      // this.ctx.fillRect(positions[0].x, positions[0].y,
      //                   positions[1].x - positions[0].x,
      //                   positions[2].y - positions[0].y);

//      this.ctx.beginPath();
//      this.ctx.arc(screenCoords.x, screenCoords.y, 2, 0, Math.PI*2, true);
//      this.ctx.fill();
    }
  }
}

MeshView.prototype.getWorldCoords = function(screen_x, screen_y) {
  var width = this.container.clientWidth;
  var height = this.container.clientHeight;
  var widthHalf = width / 2, heightHalf = height / 2;

  var vector = new THREE.Vector3((screen_x - widthHalf) / widthHalf,
                                 -(screen_y - heightHalf) / heightHalf,
                                 -1);

  var projector = new THREE.Projector();
  projector.unprojectVector(vector, this.camera);

  vector.x = (vector.x - this.model_position.x) / this.model_scale - this.control_pt_adjustment.x;
  vector.y = (vector.y - this.model_position.y) / this.model_scale - this.control_pt_adjustment.y;
  vector.z = 0;
  return vector;
}

MeshView.prototype.getScreenCoords = function(in_vec) {
  var width = this.container.clientWidth;
  var height = this.container.clientHeight;
  var widthHalf = width / 2, heightHalf = height / 2;

  var vector = new THREE.Vector3(
    (in_vec.x + this.control_pt_adjustment.x) * this.model_scale + this.model_position.x,
    (in_vec.y + this.control_pt_adjustment.y) * this.model_scale + this.model_position.y,
    in_vec.z * this.model_scale + this.model_position.z);
  var projector = new THREE.Projector();
  projector.projectVector(vector, this.camera);
//.setFromMatrixPosition(this.camera.matrixWorld), this.camera);

  vector.x = ( vector.x * widthHalf ) + widthHalf;
  vector.y = - ( vector.y * heightHalf ) + heightHalf;
  return vector;
}

MeshView.prototype.setProjCamera = function() {
  this.camera = this.proj_camera;

  return;

  // var width = this.container.clientWidth;
  // var height = this.container.clientHeight;
  // this.camera = new THREE.PerspectiveCamera( 35, width / height, 1, 15 );
  // this.camera.position.set(0, 1.0, 1.3);
  // this.cameraTarget = new THREE.Vector3( 0, 0.4, 0);

  // if (this.controls) {
  //   this.controls.object = this.camera;
  //   this.controls.center = this.cameraTarget;
  // }
}

MeshView.prototype.setOrthoCamera = function() {
  var width = this.container.clientWidth;
  var height = this.container.clientHeight;

  var max_dim = (width > height) ? width : height;
  var x_factor = width / max_dim;
  var y_factor = height / max_dim;

  this.camera =
    new THREE.OrthographicCamera(
        -0.8 * x_factor, 0.8 * x_factor,
        1.6 * y_factor, 0, 0.5, 15);
  this.camera.position.set(0, 0, 1.0);
  this.cameraTarget = new THREE.Vector3( 0, 0, 0);

  if (this.controls) {
//    this.controls.object = this.camera;
    // this.controls.center = this.cameraTarget;
  }
}

MeshView.prototype.onWindowResize = function() {
  if (!this.renderer) {
    return;
  }
  var width = $(this.container).width();
  var height = $(this.container).height();

  var canvases = $("canvas", this.container);
  canvases.css("width", width);
  canvases.css("height", height);
  for (var i = 0; i < canvases.length; ++i) {
    canvases[i].width = width;
    canvases[i].height = height;
  }

  this.renderer.setSize(width, height);

  this.camera.aspect = width / height;
  this.camera.updateProjectionMatrix();
  this.render();
}

MeshView.prototype.animate = function() {
  requestAnimationFrame(function(view) {
    return function() { view.animate() }; }(this));
  this.render();
  this.controls.update();
}

MeshView.prototype.render = function() {
  this.renderer.render(this.scene, this.camera);
  this.renderGrid(this.grid);
}

MeshView.prototype.startRendering = function() {
  this.animate();
}


// Utils -------------------------------------------------

function CreateRenderUtils() {
  this.fragment_shaders = {};
  this.loadShaders();
}

CreateRenderUtils.prototype.loadShaders = function() {
  $.get('static/red_plastic.frag', function(u){
    return function(data){u.fragment_shaders["red_plastic"] = data;};}(this));

  $.get('static/gold.frag', function(u){
    return function(data){u.fragment_shaders["gold"] = data;};}(this));

  $.get('static/blue_ceramics.frag', function(u){
    return function(data){u.fragment_shaders["blue_ceramics"] = data;};}(this));
}

CreateRenderUtils.prototype.redMaterial = function() {
  return new THREE.MeshPhongMaterial({
    ambient: 0x000000,
    color:  0xbb0000,
    specular: 0xFF4D4D,
    overdraw: true,
    shading: THREE.FlatShading,
    //      wireframe: true,
    shininess: 20
  });
};
CreateRenderUtils.prototype.shaderMaterial = function(name) {
  var uniforms = {
    amplitude: { type: "f", value: 0.0 }
  };

  var material = new THREE.ShaderMaterial({
    uniforms: uniforms,
    vertexShader: $('#vertexShader').text(),
    fragmentShader: this.fragment_shaders[name],  //$('#fragmentShader').text(),
    side: THREE.DoubleSide
  });
  return material;
};

CreateRenderUtils.prototype.createScene = function() {
  var scene = new THREE.Scene();
  scene.fog = new THREE.Fog( 0xFFFFFF, 2, 15);  // color does not matter here
  scene.add(RenderUtils.createGround());
  this.addDefaultLights(scene);
  return scene;
};
CreateRenderUtils.prototype.createRenderer = function() {
  var renderer = new THREE.WebGLRenderer({
    antialias: true,
    alpha: false
  });
  renderer.setClearColor( new THREE.Color( 0xFFFFFF ), 1);//scene.fog.color, 1 );

  renderer.gammaInput = true;
  renderer.gammaOutput = true;
  renderer.physicallyBasedShading = true;

  renderer.shadowMapEnabled = true;
  renderer.shadowMapType = THREE.PCFSoftShadowMap;
  return renderer;
};
CreateRenderUtils.prototype.addDefaultLights = function(scene) {
  scene.add(new THREE.AmbientLight( 0x999999 )); //0x888888 ) );
  //var hemLight = new THREE.HemisphereLight(0xffffff, 0xFFBF00, .4);
  //scene.add(hemLight);

  var spotLight = new THREE.SpotLight(0xbbbb00, 1, 200, 20, 10);
  spotLight.position.set(0.5, 1, -1);
  spotLight.castShadow = false;//true;
  scene.add(spotLight);

  spotLight = new THREE.SpotLight(0xbb3399, 0.8, 150, 30, 10);  //0.6
  spotLight.position.set(-2, -0.5, 0.5);
  spotLight.castShadow = false;//true;
  scene.add(spotLight);

  // Optional (best for yinyang)
  var spotLight = new THREE.SpotLight(0xbbbb00, 0.7, 200, 20, 10);
  spotLight.position.set(0.5, 1, 1);
  spotLight.castShadow = false;//true;
  scene.add(spotLight);

  scene.add(this.shadowedLight( 1, 1, 1, 0xffffff, 0.8, 0.2 ));
  scene.add(this.shadowedLight( 0.5, 1, -1, 0xffffff, 0.5, 0.08));
};
CreateRenderUtils.prototype.shadowedLight = function(x, y, z, color, intensity, darkness ) {
  var directionalLight = new THREE.DirectionalLight( color, intensity );
  directionalLight.position.set( x, y, z );

  directionalLight.castShadow = true;
  // directionalLight.shadowCameraVisible = true;

  var d = 5;
  directionalLight.shadowCameraLeft = -d;
  directionalLight.shadowCameraRight = d;
  directionalLight.shadowCameraTop = d;
  directionalLight.shadowCameraBottom = -d;

  directionalLight.shadowCameraNear = 1;
  directionalLight.shadowCameraFar = 4;

  directionalLight.shadowMapWidth = 1024;
  directionalLight.shadowMapHeight = 1024;

  //directionalLight.shadowBias = -0.005;
  directionalLight.shadowDarkness = darkness;
  return directionalLight;
};
CreateRenderUtils.prototype.createGround = function() {
  var plane = new THREE.Mesh(
    new THREE.PlaneGeometry( 40, 40 ),
    new THREE.MeshLambertMaterial({
      color: 0xFFFFFF,
    })
  );

  plane.rotation.x = -Math.PI/2;
  plane.position.y = 0.2;
  plane.receiveShadow = true;

  return plane;
};
CreateRenderUtils.prototype.computeGeometryScale = function(boundingBox) {
  var ranges = boundingBox.max.sub(boundingBox.min);
  var scale = 1.0;
  if (ranges.y > ranges.x && ranges.y > ranges.z) {
    scale = 0.5 / ranges.y;
  } else {
    var radius = Math.max(ranges.x, ranges.z);
    scale = Math.min(0.5 / ranges.y, 1.1 / radius);
  }
  return scale;
};
CreateRenderUtils.prototype.createMesh = function(geometry, material) {
  var mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = false;
  return mesh;
};


var RenderUtils = new CreateRenderUtils();
