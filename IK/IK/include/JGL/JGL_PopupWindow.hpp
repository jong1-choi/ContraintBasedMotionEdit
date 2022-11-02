//
//  JGL_PopupWindow.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/30.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_PopupWindow_hpp
#define JGL_PopupWindow_hpp

#include "JGL_Window.hpp"

namespace JGL {

struct PopupWindow: Window {
	PopupWindow();
	virtual void show() override;
	virtual void hide() override;
	virtual void render() override;
};

} // namespace JGL

#endif /* JGL_PopupWindow_hpp */
