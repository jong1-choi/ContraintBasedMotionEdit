//
//  JGL_TextInput.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/04.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_TextInput_hpp
#define JGL_TextInput_hpp

#include "JGL_TextInputBase.hpp"
#include "JGL__Valued.hpp"

namespace JGL {

struct TextInput: public TextInputBase, public _Valued<string> {
	TextInput(float x, float y, float w, float h, const string& label);
	virtual	string	value() const override { return str(); }
	virtual void	value(const string& s) override;
protected:
	
	virtual void	stringEntered() override;

	virtual bool	valid(const string& s) const override;
};

} // namespace JGL
#endif /* JGL_TextInput_hpp */
