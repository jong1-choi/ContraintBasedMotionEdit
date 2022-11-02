//
//  JGL_Aligner.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/18.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Aligner_hpp
#define JGL_Aligner_hpp

#include "JGL_LinearGroup.hpp"

namespace JGL {

struct Aligner : public LinearGroup {
	Aligner(float x=0, float y=0, float w=10, float h=10, const std::string& t="Aligner" );
	Aligner(const vec2& pos, const dim2& sz, const std::string& t="Aligner" );
	Aligner(const dim2& sz, const std::string& t="Aligner" );

	virtual float  spacing() const { return _spacing; }
	virtual void spacing( float spacing );
	virtual void rearrange(NVGcontext*,int scaling) override;

protected:
	float _spacing = 0;
	
	virtual void fitAllChildren(NVGcontext* vg, const rect& r);
	virtual void alignChildren(NVGcontext* vg, const rect& r);
	virtual bool checkResizable();
	virtual void rearrangeChildren(NVGcontext* vg, const rect& r, int scaling) override;

};


} // namespace JGL

#endif /* JGL_Aligner_hpp */
