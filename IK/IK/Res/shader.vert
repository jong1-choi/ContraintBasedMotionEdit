
#version 410 core
layout(location=0) in vec3 in_Position;
layout(location=1) in vec3 in_Normal;
uniform mat4 viewMat = mat4(1);
uniform mat4 projMat = mat4(1);
uniform mat4 modelMat = mat4(1);
uniform mat4 shadowBiasedVP = mat4(1);

out vec3 normal;
out vec4 shadowCoord;
out vec3 worldPos;
out vec3 viewNormal;
void main(void) {
	vec4 worldPos4 = modelMat* vec4( in_Position, 1. );
	normal    = normalize( (modelMat* vec4(in_Normal,0)).xyz );
	shadowCoord = shadowBiasedVP * worldPos4;
	gl_Position= projMat*viewMat* worldPos4;
	worldPos = worldPos4.xyz;
	viewNormal = (viewMat*vec4(normal,0)).xyz;
}


