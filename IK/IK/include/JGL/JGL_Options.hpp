//
//  JGL_Options.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/26.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Options_hpp
#define JGL_Options_hpp


#include "JGL_MenuPan.hpp"
#include "JGL_Property.hpp"
#include "JGL__Valued.hpp"


namespace JGL {

struct Options: public Property, public _Valued<int> {

	Options(float x,float y,float w,float h, const string& label="" );
	Options(float x,float y,float w, const string& label="" );
	Options(const vec2& pos, const dim2& sz, const string& label="" );
	Options(const vec2& pos, float w, const string& label="" );

	virtual void		add( const string& item );
	virtual void		clear();
	virtual bool		handle(int event) override;

	virtual int			value() const override { return _chosen; }
	virtual void		value(const int& i) override;

	virtual const string& option(int i) const { return _menuPan.item(i).label(); }
	virtual int 		options() const { return int(_menuPan.items()); }
	virtual string&		option(int i) { return _menuPan.item(i).label(); }
	virtual void		option(int i,const string& s) { _menuPan.item(i).label(s); }
	virtual void		disable(int i) { _menuPan.item(i).disable(); }
	virtual void		enable(int i) { _menuPan.item(i).enable(); }
	virtual bool		disabled(int i) { return _menuPan.item(i).disabled(); }
	
protected:

	virtual dim2		minActionSize(NVGcontext*vg) const override;
//	virtual bool		actionScalableW() const override { return true; };
//	virtual bool		actionScalableH() const override { return true; }
	virtual void		drawAction(NVGcontext* vg, const rect& r, int alignment) override;

	virtual float		boxHorzPadding() const { return _boxHorzPadding<0?_size_options_box_horz_padding():_boxHorzPadding; }
	static  void		chosenCallback( Widget* w, void* ud, int id );
	
	int				_chosen = -1;
	MenuPan			_menuPan;
	float   		_boxHorzPadding = -1.f;
};



} // namespace JGL

#endif /* JGL_Options_hpp */
