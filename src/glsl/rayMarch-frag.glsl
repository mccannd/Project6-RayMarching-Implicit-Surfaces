
#define MAX_GEOMETRY_COUNT 100
#define MAX_DISTANCE 50.0
#define MAX_ITERATIONS 64
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


/// --------- Matrix Ops -----------


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

mat4 transpose(in mat4 t) {
	return mat4(vec4(t[0][0], t[1][0], t[2][0], t[3][0]),
		vec4(t[0][1], t[1][1], t[2][1], t[3][1]), 
		vec4(t[0][2], t[1][2], t[2][2], t[3][2]), 
		vec4(t[0][3], t[1][3], t[2][3], t[3][3]));
}

mat4 rotY(in float theta) {

	float st = sin(theta);
	float ct = cos(theta);
	return mat4(vec4(ct, 0, st, 0), vec4(0, 1, 0, 0), 
		vec4(-st, 0, ct, 0), vec4(0, 0, 0, 1));
}

/// -------- Shapes ----------

// sphere: radius is r
float sphere(in vec4 p, in mat4 t, in float r) {
	vec4 q = inverseHomogenous(t) * p;
	return length(q.xyz) - r;
}

// box: xyz dimensions stored in r
float box(in vec4 p, in mat4 t, in vec3 r) {
	vec3 q = (inverseHomogenous(t) * p).xyz;
	vec3 d = abs(q) - r;
	return min(max(d.x, max(d.y, d.z)), 0.0) + length(max(d, 0.0));
}

// torus: r[0] is radius, r[1] is thickness
float torus(in vec4 p, in mat4 t, in vec2 r) {
	vec4 q = inverseHomogenous(t) * p;
	vec2 s = vec2(length(q.xz) - r.x, q.y);
	return length(s) - r.y;
}

// length function that leads to sharper curves
float l8(in vec2 v) {
	vec2 t = v * v;
	t = t * t;
	t = t * t;

	return pow((t.x + t.y), 0.125);
}

float pipe(in vec4 p, in mat4 t, in vec2 r) {
	vec4 q = inverseHomogenous(t) * p;
	vec2 s = vec2(length(q.xz) - r.x, q.y);
	return l8(s) - r.y;
}


float cylinder(in vec4 p, in mat4 t, in vec3 c) {
	vec4 q = inverseHomogenous(t) * p;
	vec2 d = abs(vec2(length(q.xz), q.y)) - c.y;
	return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

float plane(in vec4 p, in vec3 n, in float disp) {
	return dot(p.xyz, n) + disp;
}


/// -------- Materials -----------

//  IQ's trusty color palette
vec3 palette( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d ){
    return a + b * cos(6.28318 * (c * t + d));
}

float blinn(in vec3 n, in vec3 e, in vec3 l) {
	vec3 h = normalize(e + l);
	return max(0.0, (dot(h, n)));
}

vec3 metallicMat(in vec3 n, in vec3 e, in vec3 l) {
	float shine = blinn(n, 1.0 - e, l);
	shine = shine * shine;
	shine = shine * shine;
	shine = 0.5 * shine + 0.5 * shine * shine * shine;
	
	float ref = abs(dot(reflect(e, n), vec3(0.0, 1.0, 0.0)));
	float fres = 1.0 - abs(dot(n, e));
	fres = fres * fres;
	vec3 pal = palette(ref, vec3(0.498, 0.578, 0.708), 
		vec3(0.278, 0.158, 0.068), vec3(0.538, 0.618, 1.000), 
		vec3(0.538, 0.538, 0.667));

	return mix(vec3(shine), pal, fres);
	//return e;
	//return pal;
	//return 0.5 * n + 0.5;
	//return vec3(fres);
}


/// -------- Domain Operations ---------


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


// the entire scene of SDF objects
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

	float b = cylinder(p, m1, vec3(2.0, 1.0, 1.0));
	float s = sphere(vec4(tileXZ(pos, vec2(2.0 + sin(u_time), 2.0 + cos(u_time))), 1.0), m2, 0.5);

	float tempTime = 2.0 * fract(0.5 * u_time) - 1.0;
	float pistonTime = clamp(tempTime, 0.2, 1.0);
	float pistonDisp = sin(3.1415962 * 1.25 * (clamp(tempTime, -0.8, 0.0)));
	mat4 pistonRot = rotY(-6.283192 * 1.25 * (pistonTime - 0.2));
	pistonRot[3] = vec4(0.0, pistonDisp, 0.0, 1.0);
	float piston = box(p, pistonRot, vec3(0.5, 2.0, 0.5));

	float tor = torus(p, m2, vec2(1.0, 0.1));
	float tor2 = pipe(p, m2, vec2(3.0, 1.0));
	float ground = plane(p, vec3(0.0, 1.0, 0.0), 0.0);
	float m = smin(b, s, 0.2);
	m = smin(tor2, s, 0.2);
	m = smin(m, ground, 0.2);
	m = min(m, piston);

	return m;
}

vec3 normals(in vec3 pos) {
	return normalize(vec3(map(pos + vec3(EPSILON, 0, 0)) - map(pos - vec3(EPSILON, 0, 0)),
		map(pos + vec3(0, EPSILON, 0)) - map(pos - vec3(0, EPSILON, 0)),
		map(pos + vec3(0, 0, EPSILON)) - map(pos - vec3(0, 0, EPSILON))));
}

// sphere traces from a point to the camera to find shadowing
// soft shadow approximation from IQ
float shadow(in vec3 origin, in vec3 dir, in float limit) {
	float s = 1.0;
	//float t = 0.02;
	float step = 0.02;
	float t = 0.1;

	for (int i = 0; i < MAX_ITERATIONS; i++) {
		step = map(origin + t * dir);
		if (step < 0.001) return 0.0;
		// distance of not-quite blocking objects causes soft shadows
		s = min(s, step * 8.0 / t);
		t += step;
		if (t > limit) return s;
	}

	return s;
}

float occlusion(in vec3 origin, in vec3 normal) {
	float occ = 1.0;
	float divisor = 1.0;
	float step = 0.09;
	vec3 pos = origin;

	for(int i = 1; i < 5; i++) {
		divisor *= 2.0;
		pos = origin + step * normal;
		// adds occlusion if some object is closer than origin to ray
		float diff = float(i) * step - map(pos);
		occ -= 2.0 * diff / divisor;
	}

	return max(occ, 0.0);
}

// sphere traces from the camera in a particular direction, returns a color
vec3 trace(in vec3 origin, in vec3 dir, inout float dist) {

	// default color
	vec3 col = vec3(0.5, 0.7, 1.0);
	vec3 light = vec3(3.0, 5.0, 0.0);
	float minStep = 0.0025;
	vec3 pos = origin;

	float dt = map(origin);
	dist = 0.0;
	for(int i = 0; i < MAX_ITERATIONS; i++) {
		dist += dt;
		if (dist > MAX_DISTANCE) return col;
		pos += dt * dir;
		float nd = map(pos);
		if (nd < 0.0025) {
			//return vec3(1.0 - float(i) / float(MAX_ITERATIONS));
			//return mix(0.5 * normals(pos) + 0.5, col, dist / MAX_DISTANCE);
			vec3 ldir = normalize(light - pos);
			vec3 n = normals(pos);
			vec3 m = metallicMat(n, dir, ldir);
			float occ = occlusion(pos, n);
			float sha = 0.5 + 0.5 * shadow(pos, ldir, length(light - pos));
			//return vec3(occ);
			return mix(sha * occ * m, col, dist / MAX_DISTANCE);
			//return 0.5 * n + 0.5;
		}
		dt = nd > minStep ? nd : minStep;
	}

	return col;
}

void main() {
    // cast ray in NDC
    //vec4 cameraPos = u_camera[3];
    //vec4 ref = u_camera * vec4(0.0, 0.0, 0.1, 1.0);
	//vec4 cameraPos = vec4(15.0, 
    	//7.0 + 2.0 * sin(0.5 * u_time), 
    	//0.0, 1.0);
	
    vec4 cameraPos = vec4(15.0 * cos(u_time), 
    	7.0 + 2.0 * sin(0.5 * u_time), 
    	15.0 * sin(u_time), 1.0);
   
    vec4 ref = vec4(0.0, 0.0, 0.0, 1.0);

    float len = length(ref.xyz - cameraPos.xyz);
    vec3 F = normalize(ref.xyz - cameraPos.xyz);
    vec3 R = normalize(cross(F, vec3(0., 1., 0.)));
    vec3 U = normalize(cross(R, F));
    vec3 p = ref.xyz + u_aspect * (f_uv.x - 0.5)  * len * u_thfov * R +
    (f_uv.y - 0.5) * len * u_thfov * U;

    vec3 dir = normalize(p - cameraPos.xyz);
    // cast the ray
    float t = 0.0;
    vec3 col = trace(cameraPos.xyz, dir, t);

  
    //if (col.x < 0.0) col = vec3(1.0, 0.0, 0.0);

    //col = 0.5 * vec3(dir.x + 1.0, dir.y + 1.0, dir.z + 1.0);
    //col = vec3(dot(dir, F));

    gl_FragColor = vec4(col, 1);
}