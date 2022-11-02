//
//  JGL_Slider.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/05.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_Slider_hpp
#define JGL_Slider_hpp

#include "JGL_Property.hpp"
#include "JGL__Targettable.hpp"
#include "JGL__Valued.hpp"

namespace JGL {

const float _def_slider_padding = 8;
const float _def_slider_height = 1.5f;
const float _def_slider_tick_offset = 6;

template<typename T>
struct Slider: public Property, public _Targettable, public _Valued<T> {
	Slider(float x, float y, float w, float h, const std::string& label );

	virtual void		valueSetCallback(Callback c, void* ud=nullptr );
	virtual void		value(const T& i) override;

	virtual void		minValue(const T v);
	virtual void		maxValue(const T v);
	virtual void		range(const T& minVal, const T& maxVal);
	virtual void		autostep(bool b);
	virtual void		step(const T& s);

	virtual void		logScale(bool b);
	
	virtual void		ticked(bool b);
	virtual void		valued(bool b);

	virtual T			value() const override { return _value; }

	virtual T			minValue() const { return _minValue; }
	virtual T			maxValue() const { return _maxValue; }
	virtual bool		autostep() const { return _autostep; }
	virtual T			step() const { return _step; }

	virtual bool		logScale() const { return _logScale; }

	virtual bool		ticked() const { return _ticked; }
	virtual bool		valued() const { return _valued; }

	virtual void		rearrange(NVGcontext* vg, int scale) override;

protected:
	virtual T			xToValue( float x ) const;
	virtual float		valueToX( const T& v ) const;
	virtual void		gridDamage();
	virtual bool		gridDamaged() const { return _gridDamaged; }

	virtual void		updateGridStep();
//
	virtual void		drawAction(NVGcontext* vg, const rect& r, int alignment) override;
	virtual bool		handle(int event) override;


	virtual dim2		minActionSize(NVGcontext*vg) const override;
	virtual int			shadeState() const override;

	string				toString(const int v) const { return to_str(v); }
	string				toString(const double v) const { return to_str( roundOff(v,5) ); }

	virtual double		roundOff( double x, int precision ) const;
	virtual void		doValueSetCallback();
	
	T		_value;
	T		_minValue;
	T		_maxValue;
	T		_step;
	T		_keyStep;
	bool	_logScale = false;
	bool	_gridDamaged = false;
	bool	_pressed = false;
	Callback _valueSetCallback = nullptr;
	void*	_valueSetUserdata = nullptr;
	bool	_autostep = true;
	
	float	_gridFineStep  = 1;
	float	_gridFineOffset = 0;
	float	_gridMinorStep  = 1;
	float	_gridMinorOffset = 0;
	float	_gridMajorStep  = 1;
	float	_gridMajorOffset = 0;
	float	_gridTextStep  = 1;
	float	_gridTextOffset = 0;
	double	_gridValueStep  = 1;
	double	_gridValueOffset = 0;
	double	_gridValueBase = 1;
	
	bool	_valued = true;
	bool	_ticked = true;
	vec2 	_cursorPt;
	float	_cursorR =5;
	float	_clickedCursorFromMouse;
	
	bool	_valueTipEngaged = false;
	float	_valueTipAlpha = 0.f;
};

template<typename T> inline  void Slider<T>::drawAction(NVGcontext* vg, const rect& r, int alignment) {
	if( gridDamaged() ) {
		updateGridStep();
		_gridDamaged = false;
	}

	int s = shadeState();
	int type = (_valued?2:0)+(_ticked?1:0);

	_draw_slider_bar(vg,type,r,s);
	if( _ticked ) {
		_draw_slider_ticks_fine (vg, type, r, s, _gridFineOffset, _gridFineStep);
		_draw_slider_ticks_minor(vg, type, r, s, _gridMinorOffset, _gridMinorStep);
		_draw_slider_ticks_major(vg, type, r, s, _gridMajorOffset, _gridMajorStep);
	}
	if( _valued ) {
		nvgFontSize(vg, 7);
		nvgTextAlign(vg, ((type&1)?NVG_ALIGN_BOTTOM:NVG_ALIGN_TOP)|NVG_ALIGN_CENTER);
		nvgFillColor(vg, _color_f_label(s));
		float y = _pos_slider_number(type,r);
		double val = _gridValueOffset;
		for( float x=_gridTextOffset; x<=r.w-_size_slider_padding()*2+2; x+=_gridTextStep ) {
			double v = logScale()?pow(10,val):val;
			string str = toString( (double)v );
			nvgText(vg, x+r.x+_size_slider_padding(), y, str.c_str(), 0);
			val+=_gridValueStep;
		}
	}
	_cursorPt = vec2( valueToX( value() ), _pos_f_slider_y(type, r) );
	_cursorR = _radius_f_slider_cursor(type);
	_draw_slider_cursor(vg,type,r,_cursorPt.x,s);
	if( _valueTipEngaged ) {
		_valueTipAlpha+=0.2f;
		if( _valueTipAlpha<1 ) animate();
		else _valueTipAlpha = 1.f;
	}
	else {
		_valueTipAlpha -=0.1f;
		if( _valueTipAlpha>0 ) animate();
		else _valueTipAlpha = 0.f;
	}
	if( _valueTipAlpha>0 ) {
		nvgSave(vg);
		nvgResetScissor(vg);
		string str = toString( value() );
		dim2 sz = nvgMeasureText(vg, str, _pt_tooltip_text(),_font_tooltip_text() );
		rect rr;
		if(  0 > _cursorPt.y-sz.h-10+abs_pos().y ) {
			float r = _cursorPt.x-8, l = r-sz.w-6,
			t = _cursorPt.y-sz.h/2-1, b = t+sz.h+2;
			rr = rect(l,t,r-l,b-t);
			nvgShadowRect(vg, rr, 10, glm::vec2(0,0), _radius_tooltip_box()*1.5f);
			float R = _radius_tooltip_box();
			float RR=R*0.4477152507f;
			nvgBeginPath(vg);
			nvgMoveTo(vg,r+5,_cursorPt.y);
			nvgLineTo(vg,r,t);
			nvgLineTo(vg,l+R,t);
			nvgBezierTo(vg,l+RR,t,l,t+RR,l,t+R);
			nvgLineTo(vg,l,b-R);
			nvgBezierTo(vg,l,b-RR,l+RR,b,l+R,b);
			nvgLineTo(vg,r-R,b);
			nvgClosePath(vg);
		}
		else {
			rr = rect(_cursorPt.x-sz.w/2-3, _cursorPt.y-sz.h-10, sz.w+6, sz.h+2);
			nvgShadowRect(vg, rr, 10, glm::vec2(0,0), _radius_tooltip_box()*1.5f);
			nvgPathBalloon(vg,rr,vec2(_cursorPt.x-rr.x,rr.h+10),_radius_tooltip_box(),dim2(6,5));
			nvgTextAlign(vg, NVG_ALIGN_CENTER|NVG_ALIGN_MIDDLE);
		}
		nvgFillColor(vg, nvgColorA(_color_tooltip_box(),_valueTipAlpha));
		nvgFill(vg);
		nvgStrokeWidth(vg, _width_tooltip_bound() );
		nvgStrokeColor(vg, nvgColorA(_color_tooltip_bound(),_valueTipAlpha));
		nvgStroke(vg);
		nvgFillColor(vg, nvgColorA(_color_tooltip_text(),_valueTipAlpha));
		nvgTextAligned(vg,rr,str,NVG_ALIGN_CENTER|NVG_ALIGN_MIDDLE);
		nvgRestore(vg);
	}
}

template<typename T> inline bool Slider<T>::handle(int event) {
	switch( event ) {
		case EVENT_MOVE: {
			vec2 pt = _JGL::eventPt();
			if( length(pt-_cursorPt)<_cursorR+0.1 ) {
				_valueTipEngaged = true;
				damage();
			}
			else {
				_valueTipEngaged = false;
				damage();
			}
		} break;
		case EVENT_PUSH: {
			vec2 pt = _JGL::eventPt();
			if( length(pt-_cursorPt)<_cursorR+0.1 ) {
				_pressed =true;
				_clickedCursorFromMouse = _cursorPt.x-pt.x;
				damage();
				return true;
			}
			else {
				_pressed = false;
				value(xToValue(pt.x));
				_valueTipEngaged = true;
				doValueSetCallback();
				damage();
				return true;
			}
		}	break;
		case EVENT_DRAG:
			if( _pressed ) {
				value( xToValue(_JGL::eventPt().x+_clickedCursorFromMouse ) );
				damage();
				return true;
			} break;
		case EVENT_LEAVE:
			if( !_pressed ) {
				_valueTipEngaged = false;
				damage();
			}
			break;
		case EVENT_CANCEL:
		case EVENT_RELEASE:
			if( _pressed ) {
				_pressed = false;
				vec2 pt = _JGL::eventPt();
				if( length(pt-_cursorPt)>_cursorR+0.1 )
					_valueTipEngaged = false;
				doValueSetCallback();
				damage();
				return true;
			} break;
		case EVENT_KEYDOWN:
		case EVENT_REPEAT:
			if( _JGL::eventKey() == GLFW_KEY_LEFT ) {
				if( value()>minValue() ) {
					T v;
					if( logScale() ) {
						if(_keyStep>=_step )
							v = (T)pow(10,round( (log10(value())-_keyStep)/_keyStep )*_keyStep);
						else
							v = (T)pow(10, log10(value())-_step );
					}
					else {
						if(_keyStep>=_step )
							v = (T)(floor( (value()-_keyStep-minValue())/_keyStep )*_keyStep)+minValue();
						else
							v = value()-_step;
					}
					value(v);
					doValueSetCallback();
					damage();
					return true;
				}
			}
			else if( _JGL::eventKey() == GLFW_KEY_RIGHT ) {
				if( value()<maxValue() ) {
					T v;
					if( logScale() ) {
						if(_keyStep>=_step )
							v = (T)pow(10,round( (log10(value())+_keyStep)/_keyStep )*_keyStep);
						else
							v = (T)pow(10, log10(value())+_step );
					}
					else {
						if(_keyStep>=_step )
							v = (T)(ceil( (value()+_keyStep-minValue())/_keyStep )*_keyStep)+minValue();
						else
							v = value()+_step;
					}
					value(v);
					doValueSetCallback();
					damage();
					return true;
				}
			}
			break;
	}
	return false;
}

template<typename T> inline double Slider<T>::roundOff( double x, int precision ) const {
	double base = pow(10,glm::min(round(log10(glm::max(10e-2,(double)abs(minValue())))),
								  round(log10((double)maxValue())))-precision);
	if( logScale() )
		base = pow(10,round(log10(x))-precision);
	return round(x/base)*base;
}

template<typename T> inline void Slider<T>::doValueSetCallback() {
	if( _valueSetCallback )
		_valueSetCallback(this,_valueSetUserdata);
}

template<typename T> inline void Slider<T>::valueSetCallback(Callback c, void* ud ) {
	_valueSetCallback = c;
	_valueSetUserdata = ud;
}

template<typename T> inline void Slider<T>::ticked(bool b) {
	if( b!=_ticked ) {
		_ticked = b;
		damage();
	}
}

template<typename T> inline void Slider<T>::valued(bool b) {
	if( b!=_valued ) {
		_valued = b;
		damage();
	}
}

template<typename T> inline void Slider<T>::range(const T& minVal, const T& maxVal) {
	if( maxVal<minVal ) {
		minValue(maxVal);
		maxValue(minVal);
	}
	else {
		minValue(minVal);
		maxValue(maxVal);
	}
}

template<typename T> inline void Slider<T>::step(const T& s) {
	if( _step!=s ) {
		_step = s;
		autostep( false );
		T v;
		if( logScale() )
			v = T(pow(10,round(log10(value())/(float)_step)*_step));
		else
			v = T(round(value()/(float)_step)*_step);
		if( value()!=v )
			value(v);
		damage();
	}
}

template<typename T> inline void Slider<T>::logScale(bool b) {
	assert( typeid(T)!=typeid(int) );
	if( _logScale!=b ) {
		_logScale = b;
		if( _logScale ) {
			maxValue(maxValue());
			minValue(minValue());
		}
		damage();
		gridDamage();
	}
}

template<typename T> inline void Slider<T>::gridDamage() {
	_gridDamaged = true;
}

template<typename T> inline void Slider<T>::autostep(bool b) {
	if( _autostep != b ) {
		_autostep = b;
		if( _autostep ) {
			updateGridStep();
			value(_value);
		}
	}
}
template<typename T> inline dim2 Slider<T>::minActionSize(NVGcontext*vg) const {
	return dim2(30,25);
}

template<typename T> inline int	 Slider<T>::shadeState() const {
	int ret = Widget::shadeState();
	if( _pressed ) ret |= SHADE_PRESSED;
	return ret;
}

template<typename T> inline void Slider<T>::rearrange(NVGcontext* vg, int scale) {
	Property::rearrange(vg,scale);
	gridDamage();
}

template<typename T> inline T Slider<T>::xToValue( float x ) const {
	float sliderWidth = _actionBox.w - _size_slider_padding()*2;
	float xx = (x-(_actionBox.x+_size_slider_padding()))/sliderWidth;
	if( !logScale() )
		return T(mix( minValue(), maxValue(), xx ));
	else
		return T(pow(10,mix( log10(minValue()), log10(maxValue()), xx ) ));
}

template<typename T> float Slider<T>::valueToX( const T& v ) const {
	float sliderWidth = _actionBox.w - _size_slider_padding()*2;
	float xx;
	if( !logScale() )
		xx = (v-minValue())/float(maxValue()-minValue());
	else
		xx = float(log10(v)-log10(minValue()))/float(log10(maxValue())-log10(minValue()));
	return xx*sliderWidth + _actionBox.x+ _size_slider_padding();
}

} // namespace JGL

#endif /* JGL_Slider_hpp */
