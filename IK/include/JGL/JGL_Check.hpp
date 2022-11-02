//
//  JGL_Check.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/02.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Check_hpp
#define JGL_Check_hpp


#include "JGL_Property.hpp"
#include "JGL__Targettable.hpp"
#include "JGL__Valued.hpp"

namespace JGL {

struct Check: public Property, public _Targettable, public _Valued<bool> {
	Check(float x, float y, float w, float h, const std::string& label="" );
	Check(float x, float y, float w, const std::string& label="" );
	Check(const vec2& pos, const dim2& sz, const std::string& label="" );
	Check(const vec2& pos, float w, const std::string& label="" );

	virtual bool	value() const override { return _checked; }
	virtual void	value(const bool& i) override;
	
protected:
	virtual bool 	handle(int event) override;

	virtual dim2	minActionSize(NVGcontext*vg) const override;

	virtual void	drawAction(NVGcontext* vg, const rect& r, int alignment) override;
	virtual int		shadeState() const override;

	bool	_checked = false;
	bool	_pressed = false;
};

} // namespace JGL

#endif /* JGL_Check_hpp */
