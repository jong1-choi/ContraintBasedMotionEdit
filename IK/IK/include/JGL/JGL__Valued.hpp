//
//  JGL__Valued.hpp
//  JGL
//
//  Created by Hyun Joon Shin on 2020/10/08.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Valued_hpp
#define JGL__Valued_hpp
#include "JGL_Widget.hpp"

namespace JGL {

template<typename T>
struct _Valued {
	virtual void		value(const T&) = 0;
	virtual T			value() const = 0;
	virtual void		autoValue(T& v) { autoValue(&v); }
	virtual void		autoValue(T* v);
protected:
	T*					_autoValuePtr = nullptr;
	void				updateAutoVal();
};

template<typename T>
void	_Valued<T>::autoValue(T* v) {
	if( v ) value(*v);
	_autoValuePtr = v;
	if( _autoValuePtr )
		*_autoValuePtr = value();

}

template<typename T>
void _Valued<T>::updateAutoVal() {
	if( _autoValuePtr ) *_autoValuePtr = value();
	Widget* w = dynamic_cast<Widget*>(this);
	if( _autoValuePtr && w )
		cout<<w->label()<<" auto value is changed to :"<<(*_autoValuePtr)<<endl;
}


} // namespace JGL
#endif /* JGL__Valued_hpp */
