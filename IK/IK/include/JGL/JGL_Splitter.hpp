//
//  JGL_Splitter.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/18.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Splitter_hpp
#define JGL_Splitter_hpp

#include "JGL_Aligner.hpp"

namespace JGL {


struct Splitter: public Aligner {
	Splitter(float x,float y,float w,float h, const std::string& label="Splitter" );
	Splitter(const vec2& pos, const dim2& sz, const std::string& label="Splitter" );
	virtual float spacing() const override { return _spacing<0?_size_splitter_spacing():_spacing; }

protected:

	virtual bool handle( int event ) override;
	virtual void drawContents(NVGcontext* vg,const glm::rect& r, int a) override;

	int _onSplitter = 0;
	vec2 _lastPointer;
};

} // namespace JGL

#endif /* JGL_Splitter_hpp */
