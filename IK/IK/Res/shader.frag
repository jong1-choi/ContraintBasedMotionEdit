#version 410 core

in vec3 normal;
in vec3 viewNormal;
in vec4 shadowCoord;
in vec3 worldPos;

uniform vec4 color = vec4(1);
uniform mat4 viewMat = mat4(1);
uniform vec3 lightPos = vec3(100,100,100);
uniform vec3 lightDir = vec3(-1,-1,-1);
uniform float roughness = 0.3f;
uniform float specularFactor = 1;

uniform float cosLightFov = 0.3;
uniform int shadowEnabled = 1;
uniform sampler2D shadowMap;
uniform float shadowZNear = 10;
uniform float shadowZFar = 1000;
uniform mat4 shadowProj;
uniform vec2 lightRadius = vec2(10);

uniform vec3 iblCoeffs[9];
uniform float iblIntensityFactor = .0;

const int N_SHADOW_SAMPLE = 64;
const int N_BLOCKER_SAMPLE = 32;

out vec4 out_Color;
const float PI = 3.1415926;

float D_GGX(float NoH, float a) {
	//	float a2 = a * a;
	float a2 = a;
	float f = (NoH * a2 - NoH) * NoH + 1.0;
	return a2 / (PI * f * f);
}

vec3 F_Schlick(float u, vec3 f0) {
	return f0 + (vec3(1.0) - f0) * pow(1.0 - u, 5.0);
}

float V_SmithGGXCorrelated(float NoV, float NoL, float a) {
	//	float a2 = a * a;
	float a2 = a;
	float GGXL = NoV * sqrt((-NoL * a2 + NoL) * NoL + a2);
	float GGXV = NoL * sqrt((-NoV * a2 + NoV) * NoV + a2);
	return 0.5 / (GGXV + GGXL);
}

float Fd_Lambert() {
	return 1.0 / PI;
}

vec3 BRDF(vec3 n, vec3 l, vec3 v, float perceptualRoughness,
		  vec3 diffuseColor, float specFactor, vec3 f0 ) {
	vec3 h = normalize(v + l);
	
	float NoV = abs(dot(n, v)) + 1e-5;
	float NoL = clamp(dot(n, l), 0.0, 1.0);
	float NoH = clamp(dot(n, h), 0.0, 1.0);
	float LoH = clamp(dot(l, h), 0.0, 1.0);
	
	// perceptually linear roughness to roughness (see parameterization)
	float roughness = perceptualRoughness * perceptualRoughness;
	
	float D = D_GGX(NoH, roughness);
	vec3  F = F_Schlick(LoH, f0);
	float V = V_SmithGGXCorrelated(NoV, NoL, roughness);
	
	// specular BRDF
	vec3 Fr = clamp( (D * V) * F, vec3(0), vec3(1) );
	
	// diffuse BRDF
	vec3 Fd = diffuseColor * Fd_Lambert();
	return Fr*specFactor+Fd;
}

vec3 evalSphericalHarmonic( vec3 n ) {
	const float iblRotationTheta = 0.;
	float c = cos( iblRotationTheta ), s = sin( iblRotationTheta );
	vec2 rr= mat2(c, -s, s, c)*n.xz;
	float x = rr.x;
	float y = n.y;
	float z = rr.y;
	
	vec3 col= iblCoeffs[8]*(x*x-y*y)
	+ iblCoeffs[6]*z*z
	+ iblCoeffs[0]
	- 0.3333328848*iblCoeffs[6]
	+ (iblCoeffs[4]*x*y+ iblCoeffs[7]*x*z + iblCoeffs[5]*y*z)
	+ (iblCoeffs[3]*x  + iblCoeffs[1]*y   + iblCoeffs[2]*z  );
	
	return col*iblIntensityFactor;
}

float rand(vec2 co){
	return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}
float linearDepth( float d ) {
	return -2.0 * shadowZNear * shadowZFar / (shadowZFar + shadowZNear - (2.0 * d - 1.0) * (shadowZFar - shadowZNear));
}
vec3 project( mat4 proj, vec3 p ) {
	vec4 pos = proj*vec4(p,1);
	return pos.xyz/pos.w;
}


vec2 vogelSample( int i, int cnt, float offset) {
	const float GoldenAngle = 2.4;
	float idx = float(i);
	float r = sqrt(idx+0.5)/sqrt(float(cnt));
	float theta = idx*GoldenAngle+offset;
	return vec2( cos(theta), sin( theta ) )*r;
}

float sampleShadow(vec3 sCoord, vec2 rad, float bias ) {
	float ratio = 0.;
	float theta = rand( gl_FragCoord.xy )*PI;
//	float bias = 0.0001/* * tan(acos(dot(N,L)))*/;
	vec2 r = max( rad, vec2(0.001) );
	for (int i=0;i<N_SHADOW_SAMPLE;i++)
		if( texture( shadowMap, sCoord.xy+ rad*vogelSample(i,N_SHADOW_SAMPLE,theta) ).x < sCoord.z-bias )
			ratio += 1.;
	return smoothstep(0.,.75,1.-ratio/float(N_SHADOW_SAMPLE));
}

float blockerSearch( vec3 sCoord, vec2 searchR, float bias ) {
	float randomOffset = rand( gl_FragCoord.xy )*PI;
	
	float blockerDepth = 0.;
	int numBlockers = 0;
	for( int i = 0; i < N_BLOCKER_SAMPLE; ++i ) {
		vec2 offset = vogelSample( i, N_BLOCKER_SAMPLE, randomOffset )*searchR;
		float shadowMapDepth = texture( shadowMap, sCoord.xy + offset ).r;
		if ( shadowMapDepth < sCoord.z-bias ) {
			blockerDepth += shadowMapDepth;
			numBlockers++;
		}
	}
	if (numBlockers < 1 ) return 1.0;
	return blockerDepth / float(numBlockers);
}

float PCSS( vec3 sCoord, vec2 radius, float bias ) {
	float zEye = linearDepth( sCoord.z );
	vec2 searchR = radius/vec2(textureSize( shadowMap, 0 ))*(zEye - shadowZNear) / zEye;
	float blockerDepth = blockerSearch( sCoord, searchR, bias );
	if( shadowCoord.z<blockerDepth ) return 1.;
	blockerDepth = linearDepth( blockerDepth );
	vec2 radiusInWorld = radius * (zEye - blockerDepth) / blockerDepth;
	vec2 filterRadius = project( shadowProj, vec3( radiusInWorld, -shadowZNear) ).xy*.5;
	return sampleShadow( sCoord, filterRadius, bias );
}

void main(void) {
	vec3 N = normalize( normal );
	if( viewNormal.z <0 ) N = -N;
	vec3 dl = worldPos-lightPos;
	vec3 L = normalize( -dl );
	float visibility = 1.0;

	if( shadowEnabled>0 ) {
		visibility = PCSS( shadowCoord.xyz/shadowCoord.w, lightRadius, 0.0001 );
		visibility *= smoothstep( cosLightFov, cosLightFov+.05, dot( normalize( -L ), lightDir ));
	}

	vec3 c = BRDF( N, L, (inverse(viewMat)*vec4(0,0,1,0)).xyz, roughness, color.rgb, specularFactor, vec3(0.046) );
	vec3 am= color.rgb * evalSphericalHarmonic( N ).rgb / PI*2;

	float lightInt = max(0,dot(N,L))*visibility*PI/dot(dl,dl)*80000;
	out_Color = vec4( pow(c*lightInt+am,vec3(1/2.2)), color.a);
}
