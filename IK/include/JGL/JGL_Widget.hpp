//
//  JGL_Widget.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Widget_hpp
#define JGL_Widget_hpp

#include "JGL.hpp"
#include "JGL__Theme.hpp"
#include <string>

namespace JGL {
struct Widget;
struct Window;
struct Group;
typedef unsigned char byte;

typedef void (*Callback)(Widget*, void* owner);
typedef void (*DNDCallback)(Widget*,void* ud, const std::string& filename );

struct Widget {

// This make the widget matches to the desiredRect
	virtual void		reform(NVGcontext* vg, int scaling);
	virtual void		minSize(const dim2& sz);

// Following function can be called to change the properties of the Widget
	//! \brief Setting Top Padding
	//!
	virtual void		topPadding(float v);
	virtual void		bottomPadding(float v);
	virtual void		leftPadding(float v);
	virtual void		rightPadding(float v);
	virtual void		horzPadding(float v){ leftPadding(v); rightPadding(v); }
	virtual void		horzPadding(float l,float r){ leftPadding(l); rightPadding(r); }
	virtual void		vertPadding(float v){ topPadding(v); bottomPadding(v); }
	virtual void		vertPadding(float t,float b){ topPadding(t); bottomPadding(b); }
	virtual void		padding(float v){ horzPadding(v); vertPadding(v); }
	virtual void		padding(float h, float v){ horzPadding(h); vertPadding(v); }
	virtual void		padding(float l, float r, float t, float b){ leftPadding(l); rightPadding(r); topPadding(t); bottomPadding(b); }

	virtual void		alignment(int a);
	virtual void		autoscale(int b);

	virtual void		labelColor(const NVGcolor& c);
	virtual void		label(const string& s);
	virtual void		tooltip(const string& s);
	virtual void		labelPt(float pt);
	
	virtual void		image(const string& fn);
	virtual void		image(byte* ud, int w, int h);
	virtual void		imageFit(bool b);
	virtual void		fitImage();
	virtual void		fillImage();

	
// This does not change the size of widget directly
	virtual void		resize(const vec2& p,const dim2& sz);
	virtual void		resize(const rect& r) { resize(r.tl(),r.wh() ); }
	virtual void		size(const dim2& sz);
	virtual void		position(const vec2& p);
	virtual void		x(float v){ position(vec2(v,y())); }
	virtual void		y(float v){ position(vec2(x(),v)); }
	virtual void		w(float v){ size(dim2(v,h())); }
	virtual void		h(float v){ size(dim2(w(),v)); }
	
// Action settings
	virtual void		activate();
	virtual void		deactivate();
	virtual void		hide();
	virtual void		show();

// animated flag is not set to the widget
// calling animate() will notify the window so it should be animated
	virtual void		animate();
	virtual void		change();
	virtual void		damage();
	virtual void		redraw() { damage(); }

	virtual void		callback( Callback callback, void* userdata=nullptr );
	virtual void		dndCallback( DNDCallback cb, void* ud=nullptr );

	virtual void 		parent( Widget* g );
	virtual Window*		window();
	virtual bool		containing(const Widget* w) const { return w==this; } 
	
	
	
// Information retrival functions
// Those functions are to be used in the enclousers or subclasses
	virtual rect		bound() const { return rect(position(),size()); }
	virtual rect		contentsRect() const { return _contentsRect; }
	virtual dim2		size() const { return changed()?_setSize:_bound.wh(); }
	virtual vec2		position() const { return _bound.tl(); }
	virtual float		x() const { return _bound.x; }
	virtual float		y() const { return _bound.y; }
	virtual float		w() const { return size().w; }
	virtual float		h() const { return size().h; }

	virtual dim2		minSize() const { return _minSize; }
	virtual int			alignment() const { return _alignment; }
	virtual float		topPadding() const { return _topPadding; }
	virtual float		bottomPadding() const { return _bottomPadding; }
	virtual float		leftPadding() const { return _leftPadding; }
	virtual float		rightPadding() const { return _rightPadding; }
	virtual float		horzPadding() const { return leftPadding()+rightPadding(); }
	virtual float		vertPadding() const { return topPadding()+bottomPadding(); }

	virtual rect 		paddedRect() const { return rect(leftPadding(),topPadding(),w()-horzPadding(),h()-vertPadding()); }

	virtual NVGcolor	labelColor() const { return _labelColor.a>0.001?_labelColor:_color_label(); }
	virtual string		label() const { return _label; }
	virtual float		labelPt() const { return _labelPt>0?_labelPt:_pt_label(); }
	virtual string		tooltip() const { return _tooltip; }

	virtual int			labelImage() const { return _labelImage; }
	virtual int			labelImageWidth() const { return _labelImageSize.w; }
	virtual int			labelImageHeight() const { return _labelImageSize.h; }
	virtual idim2		labelImageSize() const { return _labelImageSize; }
	virtual bool		labelImageFit() const { return _labelImageFit; }
	virtual dim2		measureTextLabel(NVGcontext* vg) const;

	virtual bool		active() const { return !_deactivated; }
	virtual bool		changed() const { return _changed; }
	virtual bool		hidden() const { return _hidden; }
	virtual int			autoscale() const { return _autoscale; }

	virtual Callback	callback() const { return _callback; }
	virtual void*		userdata() const { return _userdata; }
	virtual DNDCallback	dndCallback() const { return _dndCallback; }
	virtual void*		dndUserdata() const { return _dndUserdata; }
	virtual Widget*		parent() const { return _parent; }


// Those function is used by the enclouser and the JGL
	
	virtual bool		under( const glm::vec2& pt ) const;
	virtual bool		under() const;
	virtual float		abs_x() const { return abs_pos().x; }
	virtual float		abs_y() const { return abs_pos().y; }
	virtual vec2		abs_pos() const;
	virtual rect		abs_rect() const {return rect( abs_pos(), size() ); }

	virtual ~Widget();
	
	virtual irect		prevViewport() const { return _prevViewport; }
	virtual irect		prevScissor() const { return _prevScrissor; }
	
	virtual void		preDrawGL();
	virtual void		postDrawGL();
	virtual void 		setViewport() const;

	Widget(float x, float y, float width, float height, const string& label="" );
	Widget(const vec2& p, const dim2& sz, const string& label="" );


protected:
	
	virtual idim2		getWindowSize() const;
	virtual void		doCallback();
	virtual void		doDNDCallback();
	virtual int			nvgImage(NVGcontext* vg);
	virtual NVGpaint	getImagePaint(NVGcontext* vg,const glm::rect& rect,int align);

// Functions to override
	virtual void		drawGL();
	virtual bool		handle( int event );
	virtual void		drawBoxOver(NVGcontext* vg, const glm::rect& r);
	virtual void		drawBox(NVGcontext* vg, const glm::rect& r);
	virtual void		drawContents(NVGcontext* vg, const glm::rect& r, int align );

// This make the widget matches to the desiredRect
	virtual void		rearrange(NVGcontext* vg, int scaling);
	virtual void		updateContentsRect(NVGcontext* vg);

	virtual int		 	shadeState() const;
	virtual void		drawImageLabel(NVGcontext* vg, const glm::rect& r, int align );
	virtual void		drawTextLabel(NVGcontext* vg, const glm::rect& r, int align );
	virtual void		draw(NVGcontext* vg);


// Geometric configuration
	rect		_bound;
	rect		_contentsRect;

	dim2		_setSize;
	dim2		_minSize			= dim2(10,10);
	float		_topPadding			= 0;
	float		_bottomPadding		= 0;
	float		_rightPadding		= 0;
	float		_leftPadding		= 0;
	int			_autoscale			= AUTOSCALE_NONE;
	int			_alignment			= ALIGN_NONE;

// Label state
	NVGcolor	_labelColor			= {0,0,0,0};
	float		_labelPt			= -1.f;
	string		_label				= "";
	string		_tooltip			= "";

// Image label state
	int			_labelImage			= -1;
	string		_labelImageFilename	= "";
	idim2		_labelImageSize		= idim2(0,0);
	byte*		_labelImageData		= nullptr;
	bool		_labelImageChanged	= false;
	bool		_labelImageFit		= false;

// Action states
	bool		_deactivated		= false;
	bool		_hidden				= false;
	bool		_changed			= false;

	Callback 	_callback			= nullptr;
	void*		_userdata 			= nullptr;

	DNDCallback _dndCallback		= nullptr;
	void*		_dndUserdata 		= nullptr;
	
	Widget*		_parent				= nullptr;


	glm::irect	_prevViewport;
	glm::irect	_prevScrissor;
	int			_scissorEnabled		= false;
	
	

	friend		_JGL;
	friend		Group;
};

} // namespace JGL

#endif /* JGL_Widget_hpp */
