const THREE = require('three');
const EffectComposer = require('three-effectcomposer')(THREE)

import {PROXY_BUFFER_SIZE} from './proxy_geometry'

var clock = new THREE.Clock();
var t = 0.0;
var useCPUCamera = false;
var useShadows = false

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
            },

            u_view: {
                type: 'm3',
                value: new THREE.Matrix3()
            },
            u_useShadow: {
                type: 'i',
                value: useShadows
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
            var F = new THREE.Vector3(0, 0, -1);
            camera.updateMatrixWorld();

            if (useCPUCamera) {
                F.applyQuaternion(camera.quaternion);
                shaderPass.material.uniforms.u_camera.value = camera.matrix;
            } else {
                var pos = new THREE.Vector3(15.0 * Math.cos(t), 
                    7.0 - 2.0 * Math.sin(0.5 * t), 15.0 * Math.sin(t));
                F = F.subVectors(new THREE.Vector3(0, 0, 0), pos).normalize();
                shaderPass.material.uniforms.u_camera.value = (new THREE.Matrix4()).makeTranslation(pos.x, pos.y, pos.z);
            }

            var R = (new THREE.Vector3(0, 0, 0)).crossVectors(F, new THREE.Vector3(0, 1, 0)).normalize();
            var U = (new THREE.Vector3(0, 0, 0)).crossVectors(R, F).normalize();
            var view = new THREE.Matrix3();
            view.set(R.x, U.x, F.x, R.y, U.y, F.y, R.z, U.z, F.z);
  
            shaderPass.material.uniforms.u_view.value = view;
            shaderPass.material.uniforms.u_aspect.value = window.innerWidth / window.innerHeight;
            t += clock.getDelta();
            shaderPass.material.uniforms.u_time.value = t;
        },

        toggleCamera: function() {
            useCPUCamera = !useCPUCamera;
        },

        toggleShadows: function() {
            useShadows = !useShadows;
            shaderPass.material.uniforms.u_useShadow.value = useShadows;
        }
    }
}