
#define MAX_GEOMETRY_COUNT 100
#define MAX_DISTANCE 50.0
#define MAX_ITERATIONS 64
#define EPSILON 0.00001


uniform vec4 u_buffer[MAX_GEOMETRY_COUNT];
uniform int u_count;
uniform mat4 u_camera;
uniform mat3 u_view;
uniform float u_thfov;
uniform float u_aspect;
uniform float u_time;
uniform int u_useShadow;

varying vec2 f_uv;

/// --------- Matrix Ops And Transformations-----------

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

// SDF Booleans:
// union is min(a, b)
// intersection is max(a, b)
// subtraction is max(-a, b)
// scaling shape f(p) is s * f(p / s)

// IQ's implementation of Blending minimum
float smin( float a, float b, float k )
{
    float h = clamp(0.5 + 0.5 *(b - a) / k, 0.0, 1.0 );
    return mix(b, a, h) - k * h * (1.0 - h);
}

/// -------- Shapes ----------
// All shapes are derived by Inigo Quilez unless noted otherwise

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

// warning: unsigned, r is the rounding blend radius
float roundBox(in vec4 p, in mat4 t, in vec3 b, in float r)
{
  vec3 q = (inverseHomogenous(t) * p).xyz;
  return length(max(abs(q)-b,0.0))-r;
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

// torus but with l8 sharpening
float pipe(in vec4 p, in mat4 t, in vec2 r) {
	vec4 q = inverseHomogenous(t) * p;
	vec2 s = vec2(length(q.xz) - r.x, q.y);
	return l8(s) - r.y;
}

// cylinder: r[0] is radius, r[1] is height
float cylinder(in vec4 p, in mat4 t, in vec2 c) {
	vec4 q = inverseHomogenous(t) * p;
	vec2 d = abs(vec2(length(q.xz), q.y)) - c;
	return min(max(d.x, d.y), 0.0) + length(max(d, 0.0)) - 0.05;
}

// plane: n is plane normal, disp is displacement
// note: one optimization for a plane is making an analytic intersection,
// but I wanted this to smoothly blend with other objects
float plane(in vec4 p, in vec3 n, in float disp) {
	return dot(p.xyz, n) + disp;
}

// Hex prism SDF by Mark Lundin
// h[0] is radius, h[1] is height
float hexPrism(in vec4 p, in mat4 t, in vec2 h) {
	vec4 q = abs(inverseHomogenous(t) * p);
	return max(q.y - h.y, max((q.x * 0.866025 + q.z * 0.5), q.z) - h.x);
}

// a composite shape
float gear(in vec4 p, in mat4 t) {
	// bounding box
	float bound = box(p, t, vec3(1.0, 0.56, 1.0));
	if (bound > 0.003) return bound;

	vec4 q = inverseHomogenous(t) * p;
	float inner = cylinder(q, mat4(1.0), vec2(0.65, 0.5));
	// spokes
	for (int i = 0; i < 4; i++) {
		mat4 r = rotY(0.7853975 * float(i));
		inner = smin(inner, roundBox(q, r, vec3(0.1, 0.4, 0.9), 0.05), 0.1);
	}

	return inner;
}


/// -------- Materials -----------

//  IQ's trusty color palette
vec3 palette( in float t, in vec3 a, in vec3 b, in vec3 c, in vec3 d ){
    return a + b * cos(6.28318 * (c * t + d));
}

// basic blinn value, not raised to a power
float blinn(in vec3 n, in vec3 e, in vec3 l) {
	vec3 h = normalize(e + l);
	return max(0.0, (dot(h, n)));
}


vec3 metallicMat(in vec3 n, in vec3 e, in vec3 l) {
	// create a diffuse shininess for main shading
	float shine = blinn(n, 1.0 - e, l);
	shine = shine * shine;
	shine = shine * shine;
	shine = 0.5 * shine + 0.5 * shine * shine * shine;
	
	// faked environment mapping from reflection
	float ref = abs(dot(reflect(e, n), vec3(0.0, 1.0, 0.0)));
	float fres = 1.0 - abs(dot(n, e));
	fres = fres * fres;
	vec3 pal = palette(ref, vec3(0.498, 0.578, 0.708), 
		vec3(0.278, 0.158, 0.068), vec3(0.538, 0.618, 1.000), 
		vec3(0.538, 0.538, 0.667));

	return mix(vec3(shine), pal, fres);
}


/// -------- Domain Operations ---------

// apply to a world space point to repeat a shape in all dimensions
vec3 tileXYZ(in vec3 p, in vec3 t) {
	return mod(p, t) - 0.5 * t;
}

// apply to a world space point to repeat a shape across the xy plane
vec3 tileXZ(in vec3 p, in vec2 t) {
	return vec3(mod(p.x, t.x) - 0.5 * t.x, p.y, mod(p.z, t.y) - 0.5 * t.y);
}

// -------- Scene -----------
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


	float s = sphere(vec4(tileXZ(pos, vec2(2.0 + sin(u_time), 2.0 + cos(u_time))), 1.0), m2, 0.5);
	float tor2 = pipe(p, m2, vec2(3.0, 1.0));
	float ground = plane(p, vec3(0.0, 1.0, 0.0), 0.0);
	
	float m = smin(tor2, s, 0.2);
	m = smin(m, ground, 0.2);

	// sawtooth timer to allow for distinct transforms
	float tempTime = 2.0 * fract(0.5 * u_time) - 1.0;
	float pistonTime = clamp(tempTime, 0.2, 1.0);
	float pistonDisp = sin(3.1415962 * 1.25 * (clamp(tempTime, -0.8, 0.0)));
	mat4 pistonRot = rotY(-3.1415962 * 1.25 * (pistonTime - 0.2));	
	mat4 innerRot = rotY(1.25 * u_time);

	// inner gearbox
	float boundbox =  box(p, mat4(1.0), vec3(2.2, 0.8, 2.2));
	if (boundbox < 0.003) {
		// clear out tiling shapes from this space
		m = max(m, -cylinder(p, mat4(1.0), vec2(2.0, 0.8)));

		// add gears
		mat4 r1 = rotY(2.0 * u_time);
		mat4 r2 = rotY(-2.0 * u_time + 0.392669);

		r1[3] = vec4(0.8, 0.1, 0.8, 1.0);
		float g = gear(p, innerRot * r1);
		r1[3] = vec4(-0.8, 0.1, -0.8, 1.0);
		g = min(g, gear(p, innerRot * r1));

		r2[3] = vec4(-0.8, 0.1, 0.8, 1.0);
		g = min(g, gear(p,innerRot *  r2));
		r2[3] = vec4(0.8, 0.1, -0.8, 1.0);
		g = min(g, gear(p,innerRot *  r2));

		m = min(m, g);
	} else m = min(m, boundbox);

	pistonRot[3] = vec4(0.0, pistonDisp, 0.0, 1.0);

	// outer bolt shapes
	for (int i = 0; i < 6; i++) {
		float f = float(i);
		pistonRot[3] = vec4(3.0 *cos(1.047196 * f), pistonDisp,  3.0 *sin(1.047196 * f),1.0);
		float p3 = hexPrism(p, pistonRot, vec2(0.5, 2.0));
		m = min(m, p3);
	}

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
	float step = 0.002;
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

// implemented from description by IQ
float occlusion(in vec3 origin, in vec3 normal) {
	float occ = 1.0;
	float divisor = 1.0;
	float step = 0.09;
	vec3 pos = origin;

	// adds occlusion if some object is closer than origin to ray
	for(int i = 1; i < 5; i++) {
		divisor *= 2.0;
		pos = origin + step * normal;
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
		float nd = map(pos); // sample the scene

		if (nd < 0.0025) {
			// close enough, hit something
			vec3 ldir = normalize(light - pos);
			vec3 n = normals(pos);
			// color
			vec3 m = metallicMat(n, dir, ldir);
			float occ = occlusion(pos, n);
			float sha = 1.0;
			if (u_useShadow == 1) {
				sha = 0.5 + 0.5 * shadow(pos, ldir, length(light - pos));
			}
			// apply distance fog
			return mix(sha * occ * m, col, dist / MAX_DISTANCE);
			
		}
		dt = nd > minStep ? nd : minStep; // sphere step
	}

	return col;
}

void main() {
    // cast ray in NDC
    // camera position is determined CPU side
    vec4 cameraPos = u_camera[3];
    vec3 F = u_view[2];
    vec3 R = u_view[0];
    vec3 U = u_view[1];

    vec3 ref = cameraPos.xyz + 0.1 * F;

    float len = 0.1;
    vec3 p = ref.xyz + u_aspect * (f_uv.x - 0.5)  * len * u_thfov * R +
    (f_uv.y - 0.5) * len * u_thfov * U;

    vec3 dir = normalize(p - cameraPos.xyz);
    
    float t = 0.0;
    vec3 col = trace(cameraPos.xyz, dir, t);

    gl_FragColor = vec4(col, 1);
}