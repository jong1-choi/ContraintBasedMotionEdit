//
//  GLTools.hpp
//  SpringMass
//
//  Created by Hyun Joon Shin on 2021/05/09.
//

#ifndef GLTools_hpp
#define GLTools_hpp


#include <JGL/JGL.hpp>
#include <JGL/JGL_Widget.hpp>
#include <fstream>
#include <tuple>

#ifdef WIN32
typedef wchar_t CHAR_T;
#include <windows.h>
inline std::wstring utf82Unicode(const std::string& s) {
	int len = MultiByteToWideChar(CP_UTF8,0,s.c_str(),-1,NULL,0);
	wchar_t* wstr = new wchar_t[len+1];
	len = MultiByteToWideChar(CP_UTF8,0,s.c_str(),-1,wstr,len);
	wstr[len]=L'\0';
	std::wstring ws(L""); ws+=wstr;
	delete wstr;
	return ws;
}
#else
typedef char CHAR_T;
#define utf82Unicode(X) (X)
#endif

static inline void printInfoProgramLog(GLuint obj ) {
	int infologLength = 0, charsWritten  = 0;
	glGetProgramiv( obj, GL_INFO_LOG_LENGTH, &infologLength );
	if( infologLength <=0 ) return;
	std::cerr<<"Program Info:"<<std::endl;
	char *infoLog = new char[infologLength];
	glGetProgramInfoLog( obj, infologLength, &charsWritten, infoLog );
	std::cerr<<infoLog<<std::endl;
	delete [] infoLog;
}

static inline void printInfoShaderLog(GLuint obj ) {
	int infologLength = 0, charsWritten  = 0;
	glGetShaderiv ( obj, GL_INFO_LOG_LENGTH, &infologLength );
	if( infologLength <=0 ) return;
	std::cerr<<"Shader Info:"<<std::endl;
	char *infoLog = new char[infologLength];
	glGetShaderInfoLog ( obj, infologLength, &charsWritten, infoLog );
	std::cerr<<infoLog<<std::endl;
	delete [] infoLog;
}

static inline std::string readText( const std::string& fn ) {
	std::ifstream t(utf82Unicode(fn));
	if( !t.is_open() ) {
		std::cerr<<"[ERROR] Text file: "<<fn<<" is not found\n";
		return "";
	}
	std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
	return str;
}

static inline GLuint loadShader( const std::string& fn, GLuint SHADER_TYPE ) {
	std::string code = readText( fn );
	GLuint shader = glCreateShader(SHADER_TYPE);
	
	const GLchar* vshaderCode = code.c_str();
	glShaderSource	( shader, 1, &vshaderCode, nullptr );
	glCompileShader	( shader );
	printInfoShaderLog	( shader );
	
	return shader;
}
static inline GLuint buildProgram(GLuint vertShader, GLuint fragShader ) {
	GLuint prog = glCreateProgram();
	glAttachShader  ( prog, vertShader );
	glAttachShader  ( prog, fragShader );
	
	glLinkProgram( prog );
	glUseProgram ( prog );
	printInfoProgramLog( prog );
	
	return prog;
}

static inline std::tuple<GLuint,GLuint,GLuint> loadProgram( const std::string& vertShaderFn, const std::string& fragShaderFn ) {
	GLuint vert = loadShader( vertShaderFn, GL_VERTEX_SHADER );
	GLuint frag = loadShader( fragShaderFn, GL_FRAGMENT_SHADER );
	GLuint prog = buildProgram( vert, frag );
	return std::make_tuple( prog, vert, frag );
}


static inline void setUniform( GLuint prog, const std::string& name, const int& v ) {
	glUniform1i(glGetUniformLocation( prog, name.c_str() ), v);
}
static inline void setUniform( GLuint prog, const std::string& name, const float& v ) {
	glUniform1f(glGetUniformLocation( prog, name.c_str() ), v);
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::ivec2& v ) {
	glUniform2iv(glGetUniformLocation( prog, name.c_str() ), 1, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::ivec3& v ) {
	glUniform3iv(glGetUniformLocation( prog, name.c_str() ), 1, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::vec2& v ) {
	glUniform2fv(glGetUniformLocation( prog, name.c_str() ), 1, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::vec3& v ) {
	glUniform3fv(glGetUniformLocation( prog, name.c_str() ), 1, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::vec4& v ) {
	glUniform4fv(glGetUniformLocation( prog, name.c_str() ), 1, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::mat3& v ) {
	glUniformMatrix3fv(glGetUniformLocation( prog, name.c_str()), 1, 0, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::mat4& v ) {
	glUniformMatrix4fv(glGetUniformLocation( prog, name.c_str()), 1, 0, glm::value_ptr(v));
}
static inline void setUniform( GLuint prog, const std::string& name, const glm::vec3* v, int n ) {
	glUniform3fv(glGetUniformLocation( prog, name.c_str() ), n, (GLfloat*)v);
}

 

extern void drawQuad();
extern void drawSphere();
extern void drawCylinder();


extern void drawQuad( const glm::vec3& p, const glm::vec3& n, const glm::vec2& sz, const glm::vec4 color = glm::vec4(0,0,.4,1) );
extern void drawSphere( const glm::vec3& p, float r, const glm::vec4 color = glm::vec4(1,.4,0,1) );
extern void drawCylinder( const glm::vec3& p1, const glm::vec3& p2, float r, const glm::vec4 color = glm::vec4(1,0,0,1) );

#endif /* GLTools_hpp */
