//
//  JGL_LinearGroup.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/01.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_LinearGroup_hpp
#define JGL_LinearGroup_hpp

#include "JGL_Group.hpp"

namespace JGL {

struct LinearGroup: public Group {

	enum DIRECTION {
		HORIZONTAL = 0,
		VERTICAL   = 1,
	};
	bool _sorted = false;
	int _type = HORIZONTAL;
	Widget* _resizable = nullptr;

	int		type() const { return _type; }
	void	type( int t ) { _type = t; }

	virtual void add( Widget* w ) override;
	virtual void		resizable( Widget* w );
	virtual void		resizable( Widget& w ) { resizable(&w); }
	virtual Widget*		resizable() const { return _resizable; }

protected:

	LinearGroup(float x, float y, float w, float h, const std::string& label="" );
	LinearGroup(const vec2& pos, const dim2& sz, const std::string& label="" );
	virtual void		sortChildren();
};

} // namespace JGL

#endif /* JGL_LinearGroup_hpp */
