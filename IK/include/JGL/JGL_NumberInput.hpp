//
//  JGL_NumberInput.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/04.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_NumberInput_hpp
#define JGL_NumberInput_hpp

#include "JGL_TextInputBase.hpp"
#include "JGL__Valued.hpp"

namespace JGL {

template<typename T>
struct NumberInput: public TextInputBase, public _Valued<T> {
	
	NumberInput(float x, float y, float w, float h, const string& label)
	: TextInputBase(x,y,w,h,label) {}

	virtual T		value() const override { return stringToNumber( TextInputBase::str() ); }
	virtual void	value(const T& v) override;
protected:
	virtual void	stringEntered() override;
	virtual void	enterPressed() override;

	virtual bool	valid( const string& s) const override;
	
	virtual string	numberToString(const T& v) const;
	virtual T		stringToNumber(const string& s) const;
};

template<typename T> inline
void NumberInput<T>::stringEntered() {
	_Valued<T>::updateAutoVal();
}

template<typename T> inline
void NumberInput<T>::value(const T& v) {
	str( numberToString( v ) );
	_Valued<T>::updateAutoVal();
}

template<typename T> inline
void NumberInput<T>::enterPressed() {
	_str = numberToString(stringToNumber(TextInputBase::str()));
	TextInputBase::enterPressed();
}

} // namespace JGL

#endif /* JGL_NumberInput_hpp */
