//
//  JGL_Label.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/20.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Label_hpp
#define JGL_Label_hpp

#include "JGL_Widget.hpp"

namespace JGL {

struct Label : public Widget {
	Label( float x, float y, float w, float h, const std::string& name="");
	Label( const vec2& pos, const dim2& sz, const std::string& name="");
//	Size minSize() const override;
};

} // namespace JGL

#endif /* JGL_Label_hpp */
