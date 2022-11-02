//
//  JGL__Draw.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Draw_hpp
#define JGL__Draw_hpp

#include "JGL.hpp"

#include <iostream>
#include "nanovg/nanovg.h"
#include <string>

inline const NVGcolor nvgColorA(const NVGcolor& c,float a) { return nvgRGBAf(c.r,c.g,c.b,c.a*a); }
extern void nvgPathLeftRoundBox( NVGcontext* ctx, float x, float y, float w, float h, float R );
extern void nvgPathRightRoundBox( NVGcontext* ctx, float x, float y, float w, float h, float R );
extern void nvgPathTopRoundBox( NVGcontext* ctx, float x, float y, float w, float h, float R );
extern void nvgPathBottomRoundBox( NVGcontext* ctx, float x, float y, float w, float h, float R );
extern void nvgTextAligned( NVGcontext* vg, const glm::rect& r,const std::string& str, int align);
extern void nvgShadowRect(NVGcontext* ctx, const glm::rect&, float sz, const glm::vec2& offset, float R );
extern void nvgGlowBox(NVGcontext* vg, const glm::rect& r, float sz, float radius, const NVGcolor& c);

extern void nvgPathCheck(NVGcontext*vg, const glm::rect& r);

extern void nvgPathHexagon(NVGcontext*vg, const glm::rect& r, float arrowHeight);
extern void nvgPathBalloon(NVGcontext*vg, const glm::rect& r, const glm::vec2& pointer, float R, const glm::dim2& arrowHeight );
extern void nvgPathLine(NVGcontext* vg, const glm::vec2& s, const glm::vec2& e);
extern void nvgPathLineR(NVGcontext* vg, const glm::vec2& s, const glm::vec2& d);
extern void nvgLine(NVGcontext* vg, const glm::vec2& s, const glm::vec2& e);
extern void nvgLineR(NVGcontext* vg, const glm::vec2& s, const glm::vec2& d);

extern void drawDot( NVGcontext* ctx, const glm::vec2& pt, float r1, const NVGcolor& c1, float r2, const NVGcolor& c2);
extern void drawDot( NVGcontext* ctx, const glm::vec2& pt, float r1, const NVGcolor& c1);
extern void drawLine( NVGcontext* ctx, const glm::vec2& pt1, glm::vec2 pt2, float width, const NVGcolor& c1);
extern void drawRect( NVGcontext* ctx, const glm::rect& rect, float width, const NVGcolor& c1);

extern void drawRoundBox(NVGcontext* ctx, const glm::rect&, float r, const NVGcolor& fillColor, const NVGcolor& lineColor );
extern void drawRoundBox(NVGcontext* ctx, const glm::rect&, float r, const NVGcolor& fillColor );
extern void drawRoundBoxL(NVGcontext* ctx, const glm::rect&, float r, const NVGcolor& lineColor );

extern void drawCntrText(NVGcontext* ctx, const glm::rect&, const std::string& str );
extern void drawLeftText(NVGcontext* ctx, const glm::rect&, const std::string& str );
extern void drawRightText(NVGcontext* ctx, const glm::rect&, const std::string& str );

extern float nvgDashedLine(NVGcontext* ctx, const glm::vec2& s, const glm::vec2& end, float fill, float gap, float phase);

extern glm::dim2 nvgMeasureText(NVGcontext* ctx, const std::string& str, float pt=-1, const char* font=nullptr );
extern glm::vec2 currentTranslation(NVGcontext* ctx);


namespace JGL {

inline vec2 alignRect(const rect& range, const dim2& obj, int align) {
	vec2 ret = range.tl();
	if( ( align&ALIGN_TOP ) && (align&ALIGN_BOTTOM) )
		ret.y = range.y+(range.h-obj.h)/2;
	else if( align&ALIGN_TOP )
		ret.y = range.y;
	else if( align&ALIGN_BOTTOM )
		ret.y = range.y+(range.h-obj.h);
	else
		ret.y = range.y+(range.h-obj.h)/2;
	
	if( ( align&ALIGN_LEFT ) && (align&ALIGN_RIGHT) )
		ret.x = range.x+(range.w-obj.w)/2;
	else if( align&ALIGN_LEFT )
		ret.x = range.x;
	else if( align&ALIGN_RIGHT )
		ret.x = range.x+(range.w-obj.w);
	else
		ret.x = range.x+(range.w-obj.w)/2;
	return ret;
}


//extern const std::string __blit_vert_code;
//extern const std::string __blit_frag_code;

inline void glErr( const std::string& message ) {
	int err = glGetError();
	if( err != GL_NO_ERROR ) {
		printf("GL Error[%04X]: %s\n", err, message.c_str() );
	}
}

struct Program {

	Program();
	Program( Program&& pe );
	~Program();
	void clear();

	bool isUsable() const;
	void use() const;
	void unuse() const;

	int progId = 0;
	int vertId = 0;
	int fragId = 0;
	std::string vertSrc = "";
	std::string fragSrc = "";

	static unsigned int compileShader( int shaderType, const std::string& src);
	void create( const std::string& vertSrc, const std::string& fragSrc );


	void setUniform( const std::string& location, int v ) const;
	void setUniform( const std::string& location, float v ) const;
	void setUniform( const std::string& location, const glm::vec2& v ) const;
	void setUniform( const std::string& location, const glm::vec3& v ) const;
	void setUniform( const std::string& location, const glm::vec4& v ) const;
	void setUniform( const std::string& location, const glm::mat2& v ) const;
	void setUniform( const std::string& location, const glm::mat3& v ) const;
	void setUniform( const std::string& location, const glm::mat4& v ) const;
	void setUniform( const std::string& location, const std::vector<float>& v ) const;
	void setUniform( const std::string& location, const std::vector<int>& v ) const;

};


struct Window;

struct DrawContext {
	GLFWwindow* contextWindow;

	DrawContext( GLFWwindow* win )
	:contextWindow(win) {}
	static int searchContext( GLFWwindow* );
	static DrawContext* getCurrentContext();
	static std::vector<DrawContext> global_contexts;
	
	unsigned quadVboId = 0;
	unsigned quadVertId = 0;
	unsigned quadTcooId = 0;
#if 0
	unsigned lineVboId = 0;
	unsigned lineVertId = 0;
	unsigned lineTcooId = 0;

	unsigned dotVboId = 0;
	unsigned dotVertId = 0;
	unsigned dotTcooId = 0;
	unsigned dotTexId = 0;
	Program colorProgram;
	Program dashProgram;
	Program circleProgram;
#endif
	Program blitProgram;
};

inline glm::mat4 getOrthoTransform(float winW, float winH) {
	return glm::translate(glm::vec3(-1,1,0))*glm::scale(glm::vec3(2.f/winW,-2.f/winH,1));
}

extern Program& blitProgram();

struct GLQuad {
	static void init( DrawContext* );
	static void render();
	static glm::mat4 getOrthoTransform( int winW, int winH ) {
		return glm::translate(glm::vec3(-1,1,0))*glm::scale(glm::vec3(2.f/winW,-2.f/winH,1));
	}
};






/*


#if 0
extern const std::string __colored_alpha_vert_code;
extern const std::string __colored_alpha_frag_code;
extern const std::string __color_vert_code;
extern const std::string __color_frag_code;
extern const std::string __dashed_vert_code;
extern const std::string __dashed_frag_code;




extern Program& colorProgram();
extern Program& dashProgram();
extern Program& circleProgram();


struct GLLine {
	static void init( DrawContext* );
	static void render();
	static glm::mat4 getLineModelMat( const glm::vec2& s, const glm::vec2& e ) {
		glm::vec2 d = e-s;
		return translate( glm::vec3( s, 0 ) )*rotate(atan2f(d.y,d.x),glm::vec3(0,0,1))*scale( glm::vec3( length(d),1,1) );
	}
};

struct GLDot{
	static void init( DrawContext* );
	static void render();
	static glm::mat4 getCircleModelMat( const glm::vec2& cntr, float r ) {
		return glm::translate(glm::vec3(cntr,0))*glm::scale(glm::vec3(r,r,1));
	}
};

extern void drawLine( const glm::vec2& s, const glm::vec2& e, const glm::vec4& color, float width, const Size& sz);
extern float drawDashedLine( const glm::vec2& s, const glm::vec2& e, const glm::vec4& color1, const glm::vec4& color2,
							float dashPhase, float dashLength, float width, const Size& sz );
extern void drawDashedRect( const glm::vec2& tl, const glm::vec2& sz, const glm::vec4& color1, const glm::vec4& color2,
					float dashPhase, float dashLength, float width, const Size& winSz );
extern void drawDot( const glm::vec2& cntr, float radius, const glm::vec4& color, const Size& sz);



#endif
*/


} // namespace JGL

#endif /* JGL__Draw_hpp */
