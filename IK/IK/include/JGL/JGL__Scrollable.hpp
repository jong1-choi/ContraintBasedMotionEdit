//
//  JGL__Scrollable.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/27.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Scrollable_hpp
#define JGL__Scrollable_hpp

#include "JGL.hpp"

namespace JGL {

struct _Scrollable {
	virtual void	scrollRange( const dim2& screen, const rect& content, int align );
	virtual void	scrollOffset(const glm::vec2& d);
	virtual vec2	scrollOffset() const;
	virtual bool	scrollHandle(int event);
	virtual bool	scrollDrawIndicators(NVGcontext* vg);
	virtual bool	scrollEnabled() const { return _scrollEnabled; }
	virtual void	enableScroll() { _scrollEnabled = true; }
	virtual void	disableScroll() { _scrollEnabled = false; }

protected:
	float	_just_scrolled_x = -1.f;
	float	_just_scrolled_y = -1.f;
	float	_on_scroll_bar_x = 0.f;
	float	_on_scroll_bar_y = 0.f;
	vec2	_scrollOffset = glm::vec2(0,0);
	bool	_mouseOnHorzScroller = false;
	bool	_mouseOnVertScroller = false;
	vec2	_scrollClicked;
	rect	_scrollWidgetBound;
	rect	_scrollContentsBound;
	vec2	_scrollRange;
	vec2	_scrollLengthes;
	int		_align;
	bool	_scrollEnabled = true;
	

	virtual bool	horzScrollerable()const;
	virtual bool	vertScrollerable()const;
	virtual bool	mouseOnVScroller(const glm::vec2& pt)const;
	virtual bool	mouseOnHScroller(const glm::vec2& pt)const;
	virtual float	horzScrollerPos(float x)const;
	virtual float	vertScrollerPos(float y)const;
	virtual float	ptToHorzScrollVal(float y) const;
	virtual float	ptToVertScrollVal(float y) const;


	virtual void	fixScrolledAmount() ;
	virtual void	drawVertIndicators(NVGcontext* vg, const glm::rect& r)const;
	virtual void	drawHorzIndicators(NVGcontext* vg, const glm::rect& r)const;
	virtual bool	drawVertScrollBar(NVGcontext* vg, const glm::rect& r);
	virtual bool	drawHorzScrollBar(NVGcontext* vg, const glm::rect& r);

};

} // namespace JGL


#endif /* JGL__Scrollable_hpp */
