const THREE = require('three');
const EffectComposer = require('three-effectcomposer')(THREE)

import {PROXY_BUFFER_SIZE} from './proxy_geometry'

var clock = new THREE.Clock();
var t = 0.0;

export default function RayMarcher(renderer, scene, camera) {
    var composer = new EffectComposer(renderer);
    var shaderPass = new EffectComposer.ShaderPass({
        uniforms: {
            u_buffer: {
                type: '4fv',
                value: undefined
            },
            u_count: {
                type: 'i',
                value: 0
            },
            u_camera: {
                type: 'm4',
                value: camera.matrix
            },
            u_thfov: {
                type: 'f',
                value: Math.tan(0.5 * camera.fov * Math.PI / 180.0)
            },
            u_aspect: {
                type: 'f',
                value: window.innerWidth / window.innerHeight
            },

            u_time: {
                type: 'f',
                value: t
            }
        },
        vertexShader: require('./glsl/pass-vert.glsl'),
        fragmentShader: require('./glsl/rayMarch-frag.glsl')
    });
    shaderPass.renderToScreen = true;
    composer.addPass(shaderPass);

    return {
        render: function(buffer) {
            shaderPass.material.uniforms.u_buffer.value = buffer;
            shaderPass.material.uniforms.u_count.value = buffer.length / PROXY_BUFFER_SIZE;

            composer.render();
        },

        update: function() {
            camera.updateMatrixWorld();
            shaderPass.material.uniforms.u_camera.value = camera.matrix;
            
            shaderPass.material.uniforms.u_aspect.value = window.innerWidth / window.innerHeight;
            t += clock.getDelta();
            shaderPass.material.uniforms.u_time.value = t;
        }
    }
}