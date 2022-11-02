//
//  JGL_Toolbar.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Toolbar_hpp
#define JGL_Toolbar_hpp

#include "JGL_Aligner.hpp"

namespace JGL {

const float DIM_TOOLBAR_ITEM_HEIGHT = 24;
const float DIM_TOOLBAR_VERT_MARGIN = 4;
const float DIM_TOOLBAR_HORZ_MARGIN = 8;
const float DIM_TOOLBAR_ITEM_GAP = 4;
const float DIM_TOOLBAR_HEIGHT = DIM_TOOLBAR_ITEM_HEIGHT+DIM_TOOLBAR_VERT_MARGIN*2;
const float DIM_MENUBAR_HEIGHT = 20;
const float DIM_TOOLBAR_ITEM_MIN_WIDTH = 48;

struct Toolbar : public Aligner {

	Toolbar(float xx,float yy,float ww,float hh,const std::string& label="Toolbar" );
	Toolbar(float xx,float yy,float ww,const std::string& label="Toolbar" );
	Toolbar(const vec2& pos,const dim2& sz,const std::string& label="Toolbar" );
	Toolbar(const vec2& pos,float ww,const std::string& label="Toolbar" );

protected:

	virtual void drawBox(NVGcontext* vg,const glm::rect& r) override;
	bool texted;
};

} // namespace JGL
#endif /* JGL_Toolbar_hpp */
