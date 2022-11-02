//
//  JGL_Button.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Button_hpp
#define JGL_Button_hpp

#include "JGL_Widget.hpp"
#include "JGL__Valued.hpp"

namespace JGL {

enum ButtonActionType {
	BUTTON_PUSH = 0,
	BUTTON_TOGGLE = 1,
};

enum ButtonOnType {
	BUTTON_SHADE = 0,
	BUTTON_CHECK = 1,
};

struct ButtonGroup;
struct RadioButtonGroup;

struct Button: public Widget, public _Valued<bool> {
	Button(float x, float y, float w, float h, const std::string& l="" );
	Button(float x, float y, float w, const std::string& l="" );
	Button(float x, float y, const std::string& l="" );
	Button(const vec2& pos, const dim2& sz, const std::string& l="" );
	Button(const vec2& pos, float w, const std::string& l="" );
	Button(const vec2& pos, const std::string& l="" );
	
	
	virtual bool	value() const override { return _on; }
	virtual void	value(const bool& i) override;

	virtual bool	pressed() const { return _pressed; }
	
	virtual void	actionType( ButtonActionType t );
	virtual ButtonActionType actionType() const { return _actionType; }

	virtual void	onType(ButtonOnType t);
	virtual ButtonOnType onType() const { return _onType; }
	
protected:

	virtual void	drawBox(NVGcontext* vg, const glm::rect& r) override;
	virtual void	drawImageLabel(NVGcontext* vg, const glm::rect& r,int align) override;
	virtual void	drawBoxOver(NVGcontext* vg, const glm::rect& r) override;
	virtual void	updateContentsRect(NVGcontext*) override;
	virtual bool	handle( int event ) override;
	virtual int 	shadeState() const override;
	virtual void	boxType( int t ) { _boxType = t; damage(); }
	virtual void	pressed(bool p) { _pressed=p; damage(); }


	
	ButtonActionType	_actionType = BUTTON_PUSH;
	bool				_pressed = false;
	bool				_on = false;
	
	int					_boxType = BUTTON_BOX_IND;
	ButtonOnType		_onType = BUTTON_SHADE;

	friend ButtonGroup;
	friend RadioButtonGroup;
};

} // namespace JGL

#endif /* JGL_Button_hpp */
