//
//  JGL_Property.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/01.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Property_hpp
#define JGL_Property_hpp

#include "JGL_Widget.hpp"

namespace JGL {

struct Property : Widget {

	virtual int			labelPos() const { return _labelPos; }
	virtual float		labelOffset() const { return _labelOffset<-100.?_size_property_label_offset():_labelOffset; }
	virtual dim2		actionSize() const { return _actionSz; }
	virtual bool		actionSizeConstrained() const { return _actionSizeConstrained; }
	virtual int			actionAlignment() const { return _actionAlignment; }

	virtual void		labelPos(int pos);
	virtual void		labelOffset(float off);
	virtual void		actionSize(const dim2& sz);
	virtual void		actionAlignment(int a);
protected:
	
	Property(float x, float y, float w, float h, const string& label="");
	Property(const vec2& pos, const dim2& sz, const string& label="");
	int					_labelPos			= LABEL_LEFT;
	float				_labelOffset		= -101.f;
	dim2				_actionSz;
	int					_actionAlignment	= ALIGN_LEFT;
	
	rect				_actionBox;
	rect				_labelBox;
	bool				_actionSizeConstrained = false;

	virtual void		unconstrainActionSize();

	virtual void		drawContents(NVGcontext* vg, const rect& r, int alignment) override;

	virtual void 		updateContentsRect(NVGcontext* vg) override;
	virtual void		rearrange(NVGcontext* vg,int scaling) override;

	virtual dim2		minActionSize(NVGcontext* vg) const = 0;
	virtual void		drawAction(NVGcontext* vg, const rect& r, int alignment)=0;

private:
	virtual void		updateLabelAction(NVGcontext*vg);
	virtual void		autoActionSize(NVGcontext*vg);
};


} // namespace JGL

#endif /* JGL_Property_hpp */
