//
//  JGL_Group.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Group_hpp
#define JGL_Group_hpp

#include "JGL_Widget.hpp"
#include "JGL__Scrollable.hpp"

namespace JGL {

struct Group: public Widget, public _Scrollable {
	
	Group(float x, float y, float w, float h, const std::string& label="");
	Group(const vec2& pos, const dim2& sz, const std::string& label="");
	
	inline int			children() const { return int(_children.size()); }
	inline Widget*		child(int i) const { return _children[i]; }
	virtual void		add( Widget* w );
	virtual void		add( Widget& w ) { add(&w); }
	virtual void		remove( Widget* w );
	virtual void		clear( bool erase=true );
	virtual void		end();

	virtual Widget*		underMouse();
	virtual Widget*		underWidget(const glm::vec2& pt);

	virtual bool		changed() const override;

	virtual bool		containing(const Widget* w) const override;
	template<typename funcT>
	void forAllChild( funcT func ) { for( auto c: _children) func(c); }


protected:
	
	virtual void		drawContents(NVGcontext* vg, const rect& r, int alignment) override;
	virtual void		drawBox(NVGcontext* vg, const rect& r) override;
	virtual void		drawBoxOver(NVGcontext* vg, const rect& r) override;
	virtual bool		handle( int event ) override;

	virtual void		updateContentsRect(NVGcontext* vg) override;
	virtual void		rearrange(NVGcontext* vg,int scaling) override;


	virtual int			searchChild( Widget* w ) const;

	virtual bool		propagateEvent( int event );
	virtual void		recursiveDrawGL();
	virtual void		rearrangeChildren(NVGcontext* vg, const rect& r, int scaling);

	vector<Widget*>		_children;
	
	friend _JGL;
};

} // namespace JGL

#endif /* JGL_Group_hpp */
