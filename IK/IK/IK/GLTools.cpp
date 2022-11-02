//
//  GLTools.cpp
//  SpringMass
//
//  Created by Hyun Joon Shin on 2021/05/09.
//

#include "GLTools.hpp"
#include <vector>
#include <tuple>

struct RenderableMesh {
	GLuint va=0;
	GLuint vBuf=0;
	GLuint nBuf=0;
	GLuint eBuf=0;
	unsigned int nFaces=0;
	void create( const std::vector<glm::vec3>& vertices,
				const std::vector<glm::vec3>& normals,
				const std::vector<glm::uvec3>& faces ) {
		if( !va ) {
			glGenVertexArrays(1, &va);
			glBindVertexArray( va );
			
			glGenBuffers(1, &vBuf);			
			glBindBuffer(GL_ARRAY_BUFFER, vBuf);
			glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray( 0 );
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
			
			glGenBuffers(1, &nBuf);
			glBindBuffer(GL_ARRAY_BUFFER, nBuf);
			glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*normals.size(), normals.data(), GL_STATIC_DRAW);
			glEnableVertexAttribArray( 1 );
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, nullptr);
			
			glGenBuffers(1, &eBuf);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eBuf);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(glm::uvec3)*faces.size(), faces.data(), GL_STATIC_DRAW);
			
			glBindBuffer(GL_ARRAY_BUFFER,0);
			nFaces = (unsigned int)(faces.size())*3;
		}
	}
	void render() {
		glBindVertexArray( va );
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eBuf );
		glDrawElements(GL_TRIANGLES, nFaces, GL_UNSIGNED_INT, 0);
		glBindVertexArray( 0 );
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0 );
	}
};


void drawQuad() {
	static RenderableMesh mesh;
	if( !mesh.va ) {
		const std::vector<glm::vec3> v = { {-1,1,0}, {-1,-1,0}, {1,1,0}, {1,-1,0} };
		const std::vector<glm::vec3> n = { {0,0,1}, {0,0,1}, {0,0,1}, {0,0,1} };
		const std::vector<glm::vec2> t = { {0,0}, {0,1}, {1,0}, {1,1} };
		const std::vector<glm::uvec3> e = { {0,1,2}, {2,1,3} };
		mesh.create( v, n, e );
	}
	mesh.render();
}

const int N_STRIP = 15;
const int N_SLICE = 30;
const float PI = 3.1415926535;
void drawSphere() {
	static RenderableMesh mesh;
	if( !mesh.va ) {
		std::vector<glm::vec3> v;
		std::vector<glm::uvec3> e;
		
		v.push_back({0,1,0});
		for( int s = 1; s<N_STRIP; s++ ) {
			float y = cosf( s*PI/N_STRIP );
			float r = sinf( s*PI/N_STRIP );
			for( int l=0; l<N_SLICE; l++ )
				v.push_back({sinf(l*PI*2/N_SLICE)*r,y,cosf(l*PI*2/N_SLICE)*r});
		}
		v.push_back({0,-1,0});
		{
			int s1 = 1;
			for( int l=0; l<N_SLICE; l++ )
				e.push_back({0,l+s1,((l+1)%N_SLICE)+s1});
		}
		for( int s = 1; s<N_STRIP; s++ ) {
			int s0 = (s-1)*N_SLICE+1;
			int s1 = s*N_SLICE+1;
			for( int l=0; l<N_SLICE; l++ ) {
				e.push_back({l+s0,l+s1,((l+1)%N_SLICE)+s0});
				e.push_back({((l+1)%N_SLICE)+s0,l+s1,((l+1)%N_SLICE)+s1});
			}
		}
		{
			int s0 = (N_STRIP-2)*N_SLICE+1;
			int s1 = (N_STRIP-1)*N_SLICE+1;
			for( int l=0; l<N_SLICE; l++ ) 
				e.push_back({s1,((l+1)%N_SLICE)+s0,l+s0});
		}
		mesh.create( v, v, e );
	}
	mesh.render();
}


void drawCylinder() {
	static RenderableMesh mesh;
	if( !mesh.va ) {
		std::vector<glm::vec3> v;
		std::vector<glm::vec3> n;
		std::vector<glm::uvec3> e;
		
		v.push_back({0,.5,0});
		n.push_back({0,1,0});
		for( int l=0; l<N_SLICE; l++ ) {
			v.push_back({sinf(l*PI*2/N_SLICE),.5,cosf(l*PI*2/N_SLICE)});
			n.push_back({0,1,0});
		}
		for( int l=0; l<N_SLICE; l++ ) {
			glm::vec2 p = {sinf(l*PI*2/N_SLICE),cosf(l*PI*2/N_SLICE)}; 
			v.push_back({p.x,.5,p.y});
			n.push_back({p.x,0,p.y});
		}
		for( int l=0; l<N_SLICE; l++ ) {
			glm::vec2 p = {sinf(l*PI*2/N_SLICE),cosf(l*PI*2/N_SLICE)}; 
			v.push_back({p.x,-.5,p.y});
			n.push_back({p.x,0,p.y});
		}
		for( int l=0; l<N_SLICE; l++ ) {
			v.push_back({sinf(l*PI*2/N_SLICE),-.5,cosf(l*PI*2/N_SLICE)});
			n.push_back({0,-1,0});
		}
		v.push_back({0,-.5,0});
		n.push_back({0,-1,0});
		{
			int s1 = 1;
			for( int l=0; l<N_SLICE; l++ )
				e.push_back({0,l+s1,((l+1)%N_SLICE)+s1});
		}
		{
			int s0 = N_SLICE+1;
			int s1 = N_SLICE*2+1;
			for( int l=0; l<N_SLICE; l++ ) {
				e.push_back({l+s0,l+s1,((l+1)%N_SLICE)+s0});
				e.push_back({((l+1)%N_SLICE)+s0,l+s1,((l+1)%N_SLICE)+s1});
			}
		}
		{
			int s0 = N_SLICE*3+1;
			int s1 = N_SLICE*4+1;
			for( int l=0; l<N_SLICE; l++ ) 
				e.push_back({s1,((l+1)%N_SLICE)+s0,l+s0});
		}
		mesh.create( v, n, e );
	}
	mesh.render();
}

static inline GLuint getCurProgram() {
	GLint prog;
	glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
	return prog;
}

void drawQuad( const glm::vec3& p, const glm::vec3& n, const glm::vec2& sz, const glm::vec4 color ) {
	glm::vec3 s = {sz.x,sz.y,1};
	glm::vec3 axis = cross(n,glm::vec3(0,0,1));
	float l = length(axis);
	float angle = atan2f(l,n.z);
	glm::mat4 modelMat;
	if( l>0.0000001 )
		modelMat = glm::translate(p)*glm::rotate( angle, axis )*glm::scale(s);
	else
		modelMat = glm::translate(p)*glm::scale(s);
	GLuint prog = getCurProgram();
	setUniform(prog, "modelMat", modelMat );
	setUniform(prog, "color", color );
	drawQuad();
}
	
void drawSphere( const glm::vec3& p, float r, const glm::vec4 color ){
	glm::mat4 modelMat = glm::translate(p)*glm::scale(glm::vec3(r));
	GLuint prog = getCurProgram();
	setUniform(prog, "modelMat", modelMat );
	setUniform(prog, "color", color );
	drawSphere();
}
void drawCylinder( const glm::vec3& p1, const glm::vec3& p2, float r, const glm::vec4 color ) {
	glm::vec3 s = {r,length(p1-p2),r};
	glm::vec3 axis = cross(glm::vec3(0,1,0),p1-p2);
	float l = length(axis);
	float angle = atan2f(l,(p1-p2).y);
	glm::mat4 modelMat;
	if( l>0.0000001 )
		modelMat = glm::translate((p1+p2)/2.f)*glm::rotate( angle, axis )*glm::scale(s);
	else
		modelMat = glm::translate((p1+p2)/2.f)*glm::scale(s);
	GLuint prog = getCurProgram();
	setUniform(prog, "modelMat", modelMat );
	setUniform(prog, "color", color );
	drawCylinder();
}

