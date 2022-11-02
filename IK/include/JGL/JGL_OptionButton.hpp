//
//  JGL_OptionButton.hpp
//  JGL
//
//  Created by Hyun Joon Shin on 2020/10/13.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_OptionButton_hpp
#define JGL_OptionButton_hpp

#include "JGL_Button.hpp"
#include "JGL_MenuPan.hpp"

namespace JGL {

struct OptionButton: public Button {
	OptionButton(float x, float y, float w, float h, const std::string& l="" );
	OptionButton(float x, float y, float w, const std::string& l="" );
	OptionButton(float x, float y, const std::string& l="" );
	OptionButton(const vec2& pos, const dim2& sz, const std::string& l="" );
	OptionButton(const vec2& pos, float w, const std::string& l="" );
	OptionButton(const vec2& pos, const std::string& l="" );

	virtual void	add( const string& item );
	virtual void	clear();

	virtual const string& option(int i) const { return _menuPan.item(i).label(); }
	virtual int 		options() const { return int(_menuPan.items()); }
	virtual string&		option(int i) { return _menuPan.item(i).label(); }
	virtual void		option(int i,const string& s) { _menuPan.item(i).label(s); }
	virtual void		disable(int i) { _menuPan.item(i).disable(); }
	virtual void		enable(int i) { _menuPan.item(i).enable(); }
	virtual bool		disabled(int i) { return _menuPan.item(i).disabled(); }

	virtual int			optionValue() const { return _chosen; }
	virtual void		optionValue(const int i);
	
	virtual void	optionCallback( Callback cb, void* ud = nullptr);
	
	virtual void	doOptionCallback();
	
protected:

	virtual void	drawBoxOver(NVGcontext* vg, const glm::rect& r) override;
	virtual bool	handle( int event ) override;
	
	static  void	chosenCallback( Widget* w, void* ud, int id );

	Callback		_optionCB = nullptr;
	void*			_optionUD = nullptr;
	
	int				_chosen = 0;
	MenuPan			_menuPan;
};

} // namespace JGL

#endif /* JGL_OptionButton_hpp */
