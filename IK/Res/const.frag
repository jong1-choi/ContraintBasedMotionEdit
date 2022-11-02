#version 410 core

uniform vec4 color = vec4(1);
uniform mat4 viewMat = mat4(1);

out vec4 out_Color;

void main(void) {
	out_Color = vec4( pow(color.rgb,vec3(1/2.2)), color.a);
//	out_Color = vec4( vec3(gl_FragCoord.z), color.a);
}
