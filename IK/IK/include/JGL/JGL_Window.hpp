//
//  JGL_Window.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Window_hpp
#define JGL_Window_hpp

#include "JGL_Group.hpp"
#include "JGL__Popup.hpp"
#include "JGL__Targettable.hpp"

namespace JGL {

struct Window : public Group, public _PopupHandler {
	Window( float width, float height, const std::string& title );
	Window( const dim2& sz, const std::string& title );

	virtual void		show() override;
	virtual void		hide() override;
	virtual void		close();
	virtual void		label( const string& l ) override;
	virtual void		damage() override { _damaged = true; }
	virtual void		animate() override { _animated = true; }
	virtual void		clearDamaged() { _damaged = false; }
	virtual void		clearAnimated() { _animated = false; }

	virtual void		targetWidget( Widget* w );
	virtual void		clearTargetWidget();

	GLFWwindow*			glfwWindow() const { return _glfwWindow; }
	GLFWwindow*			nativeWindow() const { return _glfwWindow; }
	NVGcontext*			nvgContext() const { return _vg; }
	float				pxRatio() const { return _pxRatio; }

	virtual bool		isFocused() const { return _isFocused; }
	virtual bool		animated() const { return _animated; }
	virtual bool		damaged() const { return _damaged; }
	virtual bool		destroyed() const { return _destroyed; }

	virtual float		abs_x() const override { return 0; }
	virtual float		abs_y() const override { return 0; }
	virtual idim2		getWindowSize() const override;


	virtual bool		shoudClosed() { return glfwWindowShouldClose( _glfwWindow ); }
	virtual void		reform(NVGcontext* vg, int scaling) override;

	virtual Widget*		underWidget(const glm::vec2& pt) override;
	virtual float		underWidgetChangedTimestamp() const { return _underWidgetChangedTimestamp; }
	
	virtual Widget* 	targetWidget() const;
	
protected:
	
	virtual void		destory();
	virtual void		updateUnderWidget(const glm::vec2& pt,float timestamp);
	virtual void		dismissTooltip();
	virtual void		startTooltip();

	
	virtual void		draw(NVGcontext* vg) override;
	virtual void		render();
	virtual void		isFocused(bool a) { _isFocused = a; }

	
	
	
	GLFWwindow* 	_glfwWindow		= nullptr;
	GLFWcursor*		_currentCursor	= nullptr;
	bool			_destroyed		= true;
	bool			_isFocused		= false;
	bool			_animated		= false;
	bool			_damaged		= false;

	NVGcontext*		_vg				= nullptr;
	float			_pxRatio		= 1.f;
	
	Widget*			_currentTarget	= nullptr;

	rect			_tooltipBox;
	float			_tooltipDx		= 0;
	string			_tooltipString	= "";
	bool			_tooltipEngaged	= false;
	float			_tooltipAlpha	= 0.f;
	const Widget*	_underWidget	= nullptr;
	float			_underWidgetChangedTimestamp = 0;
	
	friend _JGL;
	
	static void nvgRegisterDefaultFonts( NVGcontext* vg, const std::string& path );

};

} // namespace JGL

#endif /* JGL_Window_hpp */
