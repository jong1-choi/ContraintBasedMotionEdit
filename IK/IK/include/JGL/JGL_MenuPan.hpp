//
//  JGL_MenuPan.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/25.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_MenuPan_hpp
#define JGL_MenuPan_hpp

#include "JGL_Widget.hpp"
#include "JGL__MenuItem.hpp"

namespace JGL {

typedef void (*MenuPanCallback)(Widget* w, void* ud, int data);

struct MenuPan: public Widget {
	MenuPan( const std::string& label="MenuPan" );

	virtual void		set( const std::vector<_MenuItem>& items );
	virtual void		add( const _MenuItem& i );
	virtual void		add( const std::vector<_MenuItem>& items );
	virtual void		clear();

	virtual void		rearrange(NVGcontext* vg,int scaling) override;

	virtual int 		checkedItem() const { return _checkedItem; }
	virtual void		checkItem(int i);
	virtual void		clearSelection() { _underMouseItem = -1; }
	virtual void		callback( MenuPanCallback cb, void* ud=nullptr);
	virtual int			items() const { return int(_items.size()); }
	virtual _MenuItem&	item(int i) { return _items[i]; }
	virtual const _MenuItem& item(int i) const { return _items[i]; }

	
	virtual float		topPadding		() const override { return _topPadding<0?_size_menupan_padding_vert():_topPadding; }
	virtual float		bottomPadding	() const override { return _bottomPadding<0?_size_menupan_padding_vert():_bottomPadding; }
	virtual float		leftPadding		() const override { return _leftPadding<0?_size_menupan_padding_horz():_leftPadding; }
	virtual float		rightPadding	() const override { return _rightPadding<0?_size_menupan_padding_horz():_rightPadding; }

protected:
	
	virtual void		updateContentsRect(NVGcontext* vg) override;

	virtual bool		handle(int event) override;
	virtual void		drawBox(NVGcontext* vg, const glm::rect& r) override;
	virtual void		draw(NVGcontext* vg) override;
	virtual int 		itemUnder( const glm::vec2& pt );


	
	int						_underMouseItem = -1;
	int						_checkedItem = -1;
	MenuPanCallback			_menuCallback = nullptr;
	std::vector<_MenuItem> _items;
	
	// TODO: Menu cascading
};



} // namespace JGL

#endif /* JGL_MenuPan_hpp */
