//
//  ModelView.hpp
//  SpringMass
//
//  Created by Hyun Joon Shin on 2021/05/12.
//

#ifndef ModelView_h
#define ModelView_h

#include "GLTools.hpp"
#include <JGL/JGL_Widget.hpp>
#include <functional>

struct FB {
	size_t w = 0;
	size_t h = 0;
	GLuint fbo = 0;
	GLuint color = 0;
	GLuint depth = 0;
	
	GLint oldVP[4];
	GLint oldFB, oldSc;
	
	static void setTexParam( GLuint minFilter = GL_LINEAR, GLuint warp = GL_CLAMP_TO_EDGE ) {
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, warp );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, warp );
	}
	
	void clearGL() {
		if( color ) glDeleteTextures( 1, &color ); color = 0;
		if( depth ) glDeleteTextures( 1, &depth ); depth = 0;
		if( fbo   ) glDeleteFramebuffers( 1, &fbo ); fbo = 0;
	}
	void create( int ww, int hh ) {
		if( ww == w && hh == h ) return;
		w = ww;
		h = hh;
		printf("FB creation: %d x %d\n", w, h );
		clearGL();
		glGenTextures( 1, &color );
		glBindTexture( GL_TEXTURE_2D, color );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, ww, hh, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		setTexParam();
		
		glGenTextures( 1, &depth );
		glBindTexture( GL_TEXTURE_2D, depth );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, ww, hh, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
		setTexParam();
		
		glGenFramebuffers( 1, &fbo );
		glGetIntegerv(GL_FRAMEBUFFER_BINDING, &oldFB );
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		
		glFramebufferTexture( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color, 0);
		glFramebufferTexture( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth, 0 );
		if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
			std::cerr<<"FBO is incomplete";
		
		glBindFramebuffer(GL_FRAMEBUFFER, oldFB);
	}
	void setToTarget() {
		glGetIntegerv(GL_VIEWPORT, oldVP );
		oldSc = glIsEnabled(GL_SCISSOR_TEST);
		glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &oldFB );
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
		glViewport(0,0,w,h);
		glDisable(GL_SCISSOR_TEST);
	}
	void restoreVP() {
		glViewport(oldVP[0], oldVP[1], oldVP[2], oldVP[3] );
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, oldFB);
		if( oldSc )
			glEnable(GL_SCISSOR_TEST);
	}
	void bindColor( GLuint prog, const std::string& name, GLuint slot ) {
		glActiveTexture( GL_TEXTURE0+slot );
		glBindTexture( GL_TEXTURE_2D, color );
		setUniform( prog, name, (int)slot );
	}
	void bindDepth( GLuint prog, const std::string& name, GLuint slot ) {
		glActiveTexture( GL_TEXTURE0+slot );
		glBindTexture( GL_TEXTURE_2D, depth );
		setUniform( prog, name, (int)slot );
	}
};


struct View3D: JGL::Widget {
	float dist = 250;
	float yaw = 0;
	float pitch = 0.3;
	float fov = 0.8f;
	glm::vec2 oldPt;
	glm::vec3 oldPt3;
	glm::vec3 sceneCenter = {0,0,0};
	glm::vec3 iblCoeffs[9];
    std::function<bool(const std::string fileName)> getFileName = [](const std::string fileName ){ return false; };
	std::function<bool(const glm::vec2& pt2, const glm::vec3&,float)> pressFunc = [](const glm::vec2& pt2, const glm::vec3& pt3, float){ return false; };
	std::function<bool(const glm::vec2& pt2, const glm::vec3&,float)> dragFunc = [](const glm::vec2& pt2, const glm::vec3& pt3,float){ return false; };
	std::function<bool(const glm::vec2& pt2, const glm::vec3&,float)> releaseFunc = [](const glm::vec2& pt2, const glm::vec3& pt3,float){ return false; };
	View3D( float x, float y, float w, float h, const std::string& n )
	: JGL::Widget( x, y, w, h, n ) {
		iblCoeffs[0]=glm::vec3(2.136610, 2.136610, 2.136610);
		iblCoeffs[1]=glm::vec3(1.200665, 1.200665, 1.200665);
		iblCoeffs[2]=glm::vec3(-0.319614, -0.319614, -0.319614);
		iblCoeffs[3]=glm::vec3(-0.718034, -0.718034, -0.718034);
		iblCoeffs[4]=glm::vec3(-0.101370, -0.101370, -0.101370);
		iblCoeffs[5]=glm::vec3(-0.183365, -0.183365, -0.183365);
		iblCoeffs[6]=glm::vec3(-0.191558, -0.191558, -0.191558);
		iblCoeffs[7]=glm::vec3(0.387255, 0.387255, 0.387255);
		iblCoeffs[8]=glm::vec3(-0.021659, -0.021659, -0.02165);
	}
	virtual bool handle( int e ) override {
		if( e == JGL::EVENT_PUSH ) {
			oldPt = JGL::_JGL::eventPt();
			auto [pt3,d] = unproject( oldPt );
			oldPt3 = pt3;
			pressFunc( oldPt, oldPt3, d );
			return true;
		}
		else if( e == JGL::EVENT_DRAG ) {
			glm::vec2 pt = JGL::_JGL::eventPt();
			auto [pt3,d] = unproject( oldPt );
			oldPt3 = pt3;
			if( dragFunc( pt, pt3, d ) ) {
				redraw();
				return true;
			}
			yaw   += (pt.x-oldPt.x)/w()*3.141592;
			pitch += (pt.y-oldPt.y)/h()*3.141592;
			oldPt = pt;
			redraw();
			return true;
		}
        else if( e == JGL::EVENT_DND ) {
            std::string name = JGL::_JGL::eventDND()[0];
            getFileName(name);
            return true;
        }
		else if( e == JGL::EVENT_RELEASE ) {
			glm::vec2 pt = JGL::_JGL::eventPt();
			auto [pt3,d] = unproject( oldPt );
			oldPt3 = pt3;
			if( releaseFunc( pt, pt3, d ) ) {
				redraw();
				return true;
			}
		}
		else if( e == JGL::EVENT_ZOOM ) {
			dist*=pow(0.8, JGL::_JGL::eventZoom());
			redraw();
			return true;
		}
		return false;
	}
	virtual glm::mat4 getViewMat() const {
		return glm::translate(glm::vec3(0,0,-dist))
		*glm::rotate(pitch,glm::vec3(1,0,0))*glm::rotate( yaw, glm::vec3(0,1,0) )
		*glm::translate(-sceneCenter);
	}
	virtual glm::mat4 getProjMat() const {
		return glm::perspective( fov, w()/h(), 10.f, 1000.f );
	}
	virtual void setViewProj( GLuint prog, const std::string& viewName="viewMat", const std::string& projName="projMat" ) {
		setUniform(prog, "projMat", getProjMat());
		setUniform(prog, "viewMat", getViewMat());
	}
	virtual void drawContents(NVGcontext* vg, const glm::rect&r, int a ) override {
	}
	
	
	
	glm::vec2 toScreen( const glm::vec3& p, const glm::mat4& modelMat=glm::mat4(1) ) const {
		glm::vec4 p4 = getProjMat()*getViewMat()*modelMat*glm::vec4(p,1);
		return ((glm::vec2(p4)/p4.w)*.5f+glm::vec2(.5))*glm::vec2(w(),-h())+glm::vec2(0,h());
	}
	std::pair<glm::vec3,float> unproject(const glm::vec2& ptS ) {
		float d = 0;
		float ratio = JGL::_JGL::eventWindow()->pxRatio();
		glm::vec2 ptw = ptS + abs_pos();
		glReadPixels( ptw.x*ratio, (getWindowSize().h-ptw.y)*ratio, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);
		glm::vec3 pt3 = glm::vec3( ptS/glm::vec2(w(),h()), d)*2.f-glm::vec3(1);
		pt3.y = -pt3.y;
		glm::vec4 ptW = inverse( getProjMat()*getViewMat() )*glm::vec4(pt3,1);
		return {glm::vec3(ptW)/ptW.w,d};
	}

	glm::vec3 unproject(const glm::vec2& ptS, float d ) {
		glm::vec3 pt3 = glm::vec3( ptS/glm::vec2(w(),h()), d)*2.f-glm::vec3(1);
		pt3.y = -pt3.y;
		glm::vec4 ptW = inverse( getProjMat()*getViewMat() )*glm::vec4(pt3,1);
		return glm::vec3(ptW)/ptW.w;
	}

	
	glm::vec3 projectN(const glm::vec3& pt3 ) {
		glm::vec4 ptS = getProjMat()*getViewMat()*glm::vec4(pt3,1);
		return glm::vec3( ptS )/ptS.w;
	}
	
};

struct ModelView : View3D {
	ModelView( float x, float y, float w, float h, const std::string& name = "" )
	: View3D( x, y, w, h, name ) {}
	bool enableShadow = true;
	glm::vec3 lightPos = {0,200,250};
	//	glm::vec3 lightPos = {10,50,30};
	GLuint renderProg=0, renderVert=0, renderFrag=0;
	GLuint const_Prog=0, const_Vert=0, const_Frag=0;
	
	std::function<void()> renderFunction = [](){};
	std::function<void()> wireFunction   = [](){};
	
	FB shadowMap;
	
	virtual void drawGL() override {
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		
		if( renderProg==0 ) {
			std::tie(renderProg,renderVert,renderFrag)= loadProgram( "shader.vert", "shader.frag" );
			std::tie(const_Prog,const_Vert,const_Frag)= loadProgram( "const.vert", "const.frag" );
		}
		
		glm::mat4 shadowV, shadowP;
		float shadowZNear=100.f, shadowZFar=10000.f;
		float shadowFov = 1.0f;
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		
		if( enableShadow ) {
			shadowMap.create(1024,1024);
			shadowMap.setToTarget();
			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			shadowV = glm::lookAt(lightPos, sceneCenter, glm::vec3(0,1,0));
			shadowP = glm::perspective(shadowFov, 1.f, shadowZNear, shadowZFar);
			glUseProgram(const_Prog);
			setUniform(const_Prog, "modelMat", glm::mat4(1));
			setUniform(const_Prog, "projMat", shadowP);
			setUniform(const_Prog, "viewMat", shadowV);
			renderFunction();
			shadowMap.restoreVP();
		}
		
		glUseProgram( renderProg );
		setUniform(renderProg, "color", glm::vec4(.8,.8,.8,1) );
		setUniform(renderProg, "modelMat",glm::mat4(1));
		setViewProj( renderProg );

		setUniform(renderProg, "lightPos", lightPos );
		setUniform(renderProg, "iblCoeffs", iblCoeffs, 9);
		
		if( enableShadow ) {
			setUniform(renderProg, "shadowEnabled", 1);
			shadowMap.bindDepth(renderProg, "shadowMap", 0);
			setUniform(renderProg, "shadowProj", shadowP );
			setUniform(renderProg, "shadowZNear", shadowZNear );
			setUniform(renderProg, "shadowZFar", shadowZFar );
			setUniform(renderProg, "lightDir", normalize( sceneCenter-lightPos ) );
			setUniform(renderProg, "cosLightFov", cosf(shadowFov/2) );
			setUniform(renderProg, "shadowBiasedVP",
					   glm::translate(glm::vec3(0.5))*glm::scale(glm::vec3(0.5))
					   *shadowP*shadowV);
		}
		else
			setUniform(renderProg, "shadowEnabled", 0);
		renderFunction();
		
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glLineWidth(.2);
		glUseProgram( const_Prog );
		setUniform(const_Prog, "color", glm::vec4(0,0,0,.2));
		setUniform(const_Prog, "modelMat", glm::mat4(1));
		setViewProj( const_Prog );
		wireFunction();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
};


#endif /* ModelView_h */
