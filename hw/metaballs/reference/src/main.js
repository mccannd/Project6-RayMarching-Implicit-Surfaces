
// Credit:
// http://jamie-wong.com/2014/08/19/metaballs-and-marching-squares/
// http://paulbourke.net/geometry/polygonise/

const THREE = require('three'); // older modules are imported like this. You shouldn't have to worry about this much
import Framework from './framework'
import {generateGradientTexture} from './gradient.js'
import LUT from './marching_cube_LUT.js'
import MarchingCubes from './marching_cubes.js'

const DEFAULT_ISO_LEVEL = 0.7;
const DEFAULT_GRID_RES = 32;
const DEFAULT_GRID_WIDTH = 10;
const DEFAULT_NUM_METABALLS = 10;
const DEFAULT_MIN_RADIUS = 0.5;
const DEFAULT_MAX_RADIUS = 1;
const DEFAULT_MAX_SPEED = 0.2;

var App = {
  grid:             undefined,
  config: {
    isolevel:       DEFAULT_ISO_LEVEL,
    gridRes:        DEFAULT_GRID_RES,
    gridWidth:      DEFAULT_GRID_WIDTH,
    gridCellWidth:  DEFAULT_GRID_WIDTH / DEFAULT_GRID_RES,
    numMetaballs:   DEFAULT_NUM_METABALLS,
    minRadius:      DEFAULT_MIN_RADIUS,
    maxRadius:      DEFAULT_MAX_RADIUS,
    maxSpeed:       DEFAULT_MAX_SPEED
  },
  camera:           undefined,
  scene:            undefined,
  renderer:         undefined,
  composer:         undefined,
  isPaused:         false,
  insPointsGrp:     new THREE.Group(),
  cellsGrp:         new THREE.Group()
};

// called after the scene loads
function onLoad(framework) {

  var {scene, camera, renderer, gui, stats} = framework;
  App.scene = scene;
  App.camera = camera;
  App.renderer = renderer;

  renderer.setClearColor( 0xbfd1e5 );
  // scene.add(new THREE.AxisHelper(20));

  setupCamera(App.camera);
  setupLights(App.scene);
  setupScene(App.scene);
  setupGUI(gui);
}

// called on frame updates
function onUpdate(framework) {
  if (App.grid) {
    App.grid.update();
  }
}

function setupCamera(camera) {
  // set camera position
  camera.position.set(5, 5, 30);
  camera.lookAt(new THREE.Vector3(0,0,0));
}

function setupLights(scene) {

  // Directional light
  var directionalLight = new THREE.DirectionalLight( 0xffffff, 1 );
  directionalLight.color.setHSL(0.1, 1, 0.95);
  directionalLight.position.set(1, 10, 2);
  directionalLight.position.multiplyScalar(10);

  scene.add(directionalLight);
}

function setupScene(scene) {
  scene.background = generateGradientTexture();
  App.grid = new MarchingCubes(App);
}

function setupGUI(gui) {

  // more information here: https://workshop.chromeexperiments.com/examples/gui/#1--Basic-Usage
  // --- CONFIG ---
  gui.add(App, 'isPaused').onChange(function(value) {
    App.isPaused = value;
    if (value) {
      App.grid.pause();
    } else {
      App.grid.play();
    }
  });

  gui.add(App.config, 'numMetaballs', 1, 10).onChange(function(value) {
    App.config.numMetaballs = value;
    App.grid.init(App);
  });

  // --- DEBUG ---

  // var debugFolder = gui.addFolder('Grid');
  // debugFolder.add(App.grid, 'showGrid').onChange(function(value) {
  //   App.grid.showGrid = value;
  //   if (value) {
  //     App.grid.show();
  //   } else {
  //     App.grid.hide();
  //   }
  // });

  // debugFolder.add(App.grid, 'showSpheres').onChange(function(value) {
  //   App.grid.showSpheres = value;
  //   if (value) {
  //     for (var i = 0; i < App.config.numMetaballs; i++) {
  //       App.grid.balls[i].show();
  //     }
  //   } else {
  //     for (var i = 0; i < App.config.numMetaballs; i++) {
  //       App.grid.balls[i].hide();
  //     }
  //   }
  // });
  // debugFolder.open();  
}

// when the scene is done initializing, it will call onLoad, then on frame updates, call onUpdate
Framework.init(onLoad, onUpdate);
