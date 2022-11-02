//
//  JGL__ThemeDark.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/02.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__ThemeDark_hpp
#define JGL__ThemeDark_hpp

#include "JGL__Theme.hpp"

namespace JGL {

struct ThemeDark: Theme {

	virtual NVGcolor  	_color_background						()const override;
	virtual NVGcolor  	_color_label							()const override;
	virtual NVGcolor 	_color_panel							()const override;
	virtual NVGcolor	_color_panel_separator					()const override;
	virtual NVGcolor	_color_list_box							()const override;
	virtual NVGcolor	_color_list_box_unfocussed				()const override;
	virtual NVGcolor	_color_button_bound						()const override;
	virtual NVGcolor	_color_button_bound_engaged				()const override;
	virtual NVGcolor	_color_button_shade_pushed				()const override;
	virtual NVGcolor	_color_button_shade_engaged				()const override;
	virtual NVGcolor	_color_button_fill						()const override;
	virtual NVGcolor	_color_button_fill_pressed				()const override;
/*	virtual NVGcolor	_color_scroll_indicator					()const override;
	virtual NVGcolor	_size_scroller_background				()const override;
*/	virtual NVGcolor	_color_toolbar_fill_unfocused			()const override;
	virtual NVGcolor	_color_toolbar_fill_line1				()const override;
	virtual NVGcolor	_color_toolbar_fill_line2				()const override;
	virtual NVGcolor	_color_toolbar_fill_grad1				()const override;
	virtual NVGcolor	_color_toolbar_fill_grad2				()const override;
	virtual NVGcolor	_color_toolbar_fill_line3				()const override;
	virtual NVGcolor	_color_toolbar_fill_line4				()const override;
	virtual NVGcolor	_color_statusbar_fill_top				()const override;
	virtual NVGcolor 	_color_menupan_background				()const override;
	virtual NVGcolor	_color_menupan_separator				()const override;
	virtual NVGcolor	_color_menupan_bound					()const override;
	virtual NVGcolor  	_color_menupan_fill_under_item			()const override;
	virtual NVGcolor	_color_menu_text						()const override;
	virtual NVGcolor	_color_menu_text_under_item				()const override;

	
	
	virtual NVGcolor	_color_f_button_bound( int s ) const override;
	virtual NVGcolor	_color_f_button_fill_over( int s ) const override;
	virtual float		_alpha_f_button_fill( int s ) const override;
	virtual NVGcolor	_color_f_button_fill( int s ) const override;
};

} // namespace JGL

#endif /* JGL__ThemeDark_hpp */
