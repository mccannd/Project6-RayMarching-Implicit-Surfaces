
#define MAX_GEOMETRY_COUNT 100
#define MAX_DISTANCE 50.0
#define MAX_ITERATIONS 50
#define EPSILON 0.00001

/* This is how I'm packing the data
struct geometry_t {
    vec3 position;
    float type;
};
*/
uniform vec4 u_buffer[MAX_GEOMETRY_COUNT];
uniform int u_count;
uniform mat4 u_camera;
uniform float u_thfov;
uniform float u_aspect;
uniform float u_time;

varying vec2 f_uv;

// inverts a homogenous transformation matrix
mat4 inverseHomogenous(in mat4 t) {
	mat3 rot = mat3(vec3(t[0][0], t[1][0], t[2][0]),
		vec3(t[0][1], t[1][1], t[2][1]), 
		vec3(t[0][2], t[1][2], t[2][2]));
	vec4 tra = t[3];
	mat4 inv = mat4(vec4(rot[0], 0.0),
		vec4(rot[1], 0.0),
		vec4(rot[2], 0.0),
		vec4(-1.0 * rot * tra.xyz, 1.0));
	return inv;
}


float sphere(in vec4 p, in mat4 t, in float r) {
	vec4 q = inverseHomogenous(t) * p;
	return length(q.xyz) - r;
}

float box(in vec4 p, in mat4 t, in vec3 r) {
	vec3 q = (inverseHomogenous(t) * p).xyz;
	vec3 d = abs(q) - r;
	return min(max(d.x, max(d.y, d.z)), 0.0) + length(max(d, 0.0));
}

float s2(in vec3 p, in float r) {
	return length(p) - r;
}


vec3 tileXYZ(in vec3 p, in vec3 t) {
	return mod(p, t) - 0.5 * t;
}

vec3 tileXZ(in vec3 p, in vec2 t) {
	return vec3(mod(p.x, t.x) - 0.5 * t.x, p.y, mod(p.z, t.y) - 0.5 * t.y);
}

// IQ's implementation of Blending
float smin( float a, float b, float k )
{
    float h = clamp(0.5 + 0.5 *(b - a) / k, 0.0, 1.0 );
    return mix(b, a, h) - k * h * (1.0 - h);
}

// the entire scene of boold objects
float map(in vec3 pos) {
	vec4 p = vec4(pos, 1.0);
	mat4 m1 = mat4(vec4(1, 0, 0, 0),
		vec4(0, 1, 0, 0),
		vec4(0, 0, 1, 0),
		vec4(1, -1, 0, 1));
	mat4 m2 = mat4(vec4(1, 0, 0, 0),
		vec4(0, 1, 0, 0),
		vec4(0, 0, 1, 0),
		vec4(0, 0, 0, 1));

	float b = box(p, m1, vec3(2.0, 1.0, 0.5));
	float s = sphere(vec4(tileXZ(pos, vec2(3.0 + 2.0 * sin(u_time), 2.0)), 1.0), m2, 0.5);

	float m = smin(b, s, 0.2);

	return m;
	//return s2(pos, 1.0);
}

vec3 trace(in vec3 origin, in vec3 dir, inout float dist) {

	// default color
	vec3 col = vec3(0, 0, 0);
	float minStep = 0.0001;
	vec3 pos = origin;

	float dt = map(origin);
	dist = 0.0;
	for(int i = 0; i < MAX_ITERATIONS; i++) {
		dist += dt;
		pos += dt * dir;
		float nd = map(pos);
		if (nd < 0.001 && nd > 0.0) {
			return vec3(1.0 - float(i) / float(MAX_ITERATIONS));
		}
		dt = nd > minStep? nd : minStep;
	}

	return col;
}

void main() {
    // cast ray in NDC
    vec4 cameraPos = u_camera[3];
    vec4 ref = u_camera * vec4(0.0, 0.0, 0.1, 1.0);

    cameraPos = vec4(15.0 * cos(u_time), 4.0 + 2.0 * sin(0.5 * u_time), 15.0 * sin(u_time), 1.0);
    ref = vec4(0.0, 0.0, 0.0, 1.0);

    float len = length(ref.xyz - cameraPos.xyz);
    vec3 F = normalize(ref.xyz - cameraPos.xyz);
    vec3 R = normalize(cross(F, vec3(0., 1., 0.)));
    vec3 U = normalize(cross(R, F));
    vec3 p = ref.xyz + u_aspect * (f_uv.x - 0.5)  * len * u_thfov * R +
    (f_uv.y - 0.5) *len * u_thfov * U;

    vec3 dir = normalize(p - cameraPos.xyz);
    // cast the ray
    float t = 0.0;
    vec3 col = trace(cameraPos.xyz, dir, t);

  
    if (col.x < 0.0) col = vec3(1.0, 0.0, 0.0);

    //col = 0.5 * vec3(dir.x + 1.0, dir.y + 1.0, dir.z + 1.0);
    //col = vec3(dot(dir, F));

    gl_FragColor = vec4(col, 1);
}