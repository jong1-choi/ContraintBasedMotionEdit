//
//  JGL_ButtonGroup.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/20.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_ButtonGroup_hpp
#define JGL_ButtonGroup_hpp

#include "JGL_Aligner.hpp"

namespace JGL {

struct ButtonGroup: public Aligner {
	ButtonGroup(float x, float y, float w, float h, const std::string& label="ButtonGroup" );
	ButtonGroup(float x, float y, float w, const std::string& label="ButtonGroup" );
	ButtonGroup(const vec2& pos, const dim2& sz, const std::string& label="ButtonGroup" );
	ButtonGroup(const vec2& pos, float w, const std::string& label="ButtonGroup" );
	
	virtual void rearrange(NVGcontext* vg,int scaling) override;
	virtual void asGroup() { _asGroup = true; change(); }
	virtual void asIndividual() { _asGroup = false; change(); }
	
protected:
	
	virtual void drawBoxOver(NVGcontext* vg,const glm::rect& r) override;
	bool _asGroup = true;

};


} // namespace JGL

#endif /* JGL_ButtonGroup_hpp */
