//
//  JGL__Theme.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/22.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Theme_hpp
#define JGL__Theme_hpp

#include "JGL.hpp"
#include "JGL__Draw.hpp"

typedef const char* fontname;


namespace JGL {

extern int _alignment_to_NVGAlign( int a );


struct Theme {

	static	void setTheme(Theme*);
	static	Theme* currentTheme;
	static	Theme* getCurrentTheme();
	
	virtual NVGcolor  	_color_background						()const;
	virtual float		_pt_label								()const;
	virtual NVGcolor  	_color_label							()const;
	virtual fontname	_font_label								()const;
	virtual NVGcolor 	_color_panel							()const;
	virtual NVGcolor	_color_panel_separator					()const;
	virtual NVGcolor	_color_list_box							()const;
	virtual NVGcolor	_color_list_box_unfocussed				()const;
	virtual NVGcolor	_color_button_bound						()const;
	virtual NVGcolor	_color_button_bound_engaged				()const;
	virtual NVGcolor	_color_button_shade_pushed				()const;
	virtual NVGcolor	_color_button_shade_engaged				()const;
	virtual NVGcolor	_color_button_fill						()const;
	virtual NVGcolor	_color_button_fill_pressed				()const;
	virtual float		_width_button_bound						()const;
	virtual fontname	_font_menuitem							()const;
	virtual float		_size_scroller_offset					()const;
	virtual float		_size_scroller_offset2					()const;
	virtual float		_size_scroller_thickness				()const;
	virtual float		_size_scroller_thickness2				()const;
	virtual NVGcolor	_color_scroll_indicator					()const;
	virtual float		_size_scroll_indicator_arrow_height 	()const;
	virtual float		_size_scroll_indicator_arrow_width		()const;
	virtual NVGcolor	_size_scroller_background				()const;
	virtual float		_size_split_min							()const;
	virtual float		_size_property_label_offset				()const;
	virtual NVGcolor	_color_toolbar_fill_unfocused			()const;
	virtual NVGcolor	_color_toolbar_fill_line1				()const;
	virtual NVGcolor	_color_toolbar_fill_line2				()const;
	virtual NVGcolor	_color_toolbar_fill_grad1				()const;
	virtual NVGcolor	_color_toolbar_fill_grad2				()const;
	virtual NVGcolor	_color_toolbar_fill_line3				()const;
	virtual NVGcolor	_color_toolbar_fill_line4				()const;
	virtual NVGcolor	_color_statusbar_fill_top				()const;
	virtual float		_width_panel_separator					()const;
	virtual NVGcolor 	_color_menupan_background				()const;
	virtual NVGcolor	_color_menupan_separator				()const;
	virtual NVGcolor	_color_menupan_bound					()const;
	virtual NVGcolor  	_color_menupan_fill_under_item			()const;
	virtual NVGcolor	_color_menu_text						()const;
	virtual NVGcolor	_color_menu_text_under_item				()const;
	virtual float		_pt_menu_text							()const;
	virtual float		_size_menupan_separator_height			()const;
	virtual float		_size_menupan_menuitem_height			()const;
	virtual float		_size_menupan_shadow_width				()const;
	virtual float		_size_menupan_shadow_offset_y			()const;
	virtual float		_radius_menupan							()const;
	virtual float		_radius_popupbox						()const;
	virtual float		_radius_button							()const;
	virtual float 		_size_button_height						()const;
	virtual float		_size_button_width						()const;
	virtual float		_size_toolbar_height					()const;
	virtual float		_size_statusbar_height					()const;
	virtual float		_size_options_box_horz_padding			()const;

	virtual float		_size_menupan_check_left_offset			()const;
	virtual float		_size_splitter_spacing					()const;
	virtual float		_size_menupan_padding_horz				()const;
	virtual float		_size_menupan_padding_vert				()const;
	
	virtual dim2		_dim_check_action_size					()const;
	virtual dim2		_dim_options_action_min_size			()const;

	virtual NVGcolor	_color_targetted_glow					()const;

	
	virtual NVGcolor	_color_f_button_bound( int s ) const;
	virtual NVGcolor	_color_f_button_fill_over( int s ) const;
	virtual float		_alpha_f_label( int s ) const;
	virtual NVGcolor	_color_f_label( int s ) const;
	virtual float		_alpha_f_button_fill( int s ) const;
	virtual NVGcolor	_color_f_button_fill( int s ) const;

	virtual void		_draw_text_label(NVGcontext* vg, const glm::rect& r, const NVGcolor& color, const std::string& str, float pt, int align, int state ) const;
	virtual glm::dim2	_measure_text_label(NVGcontext* vg, const std::string& str, float pt ) const;

	virtual void		_path_button_box( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_button_box_left( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_button_box_right( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_button_box_top( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_button_box_bottom( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_button_box_middle( NVGcontext* vg, const glm::rect& r ) const;

	virtual void		_path_imagelabel( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_draw_imagelabel( NVGcontext* vg, const glm::rect& r, const NVGpaint& paint, int state) const;
	virtual void		_path_group( NVGcontext* vg, const glm::rect& r ) const;

	virtual void		_path_scroll_indicator_top( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_scroll_indicator_bottom( NVGcontext* vg, const glm::rect& r ) const;
	virtual void		_path_scroll_indicator_left( NVGcontext* vg, const glm::rect& r )  const;
	virtual void		_path_scroll_indicator_right( NVGcontext* vg, const glm::rect& r )  const;
	virtual void		_draw_scroll_indicator_left(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_scroll_indicator_right(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_scroll_indicator_top(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_scroll_indicator_bottom(NVGcontext* vg, const glm::rect& r)  const;

	virtual void		_path_scrollbar_horz( NVGcontext* vg, const glm::rect& r, float scale, float total, float delta )  const;
	virtual void		_path_scrollbar_vert( NVGcontext* vg, const glm::rect& r, float scale, float total, float delta )  const;
	virtual void		_draw_scrollbar_horz( NVGcontext* vg, const glm::rect& r, float alpha, float sz, float total, float amount) const;
	virtual void		_draw_scrollbar_vert( NVGcontext* vg, const glm::rect& r, float alpha, float sz, float total, float amount)  const;

	virtual bool		_test_scrollbar_horz( const glm::rect& r, const glm::vec2& pt) const;
	virtual bool		_test_scrollbar_vert( const glm::rect& r, const glm::vec2& pt) const;
	virtual glm::dim2	_measure_menuitem(NVGcontext* vg, const std::string& str, bool separator ) const;

	virtual void		_draw_menuitem(NVGcontext* vg, const glm::rect& r, const std::string& str, int align, int state, bool disabled, bool separator ) const;
	virtual void		_path_separator_vert( NVGcontext* vg, const glm::rect& r, float y ) const;
	virtual void		_path_separator_horz( NVGcontext* vg, const glm::rect& r, float x ) const;
	virtual void		_path_menupan_box( NVGcontext* vg, const glm::rect& r )  const;
	virtual void		_draw_menupan_under_item_fill(NVGcontext* vg, const glm::rect& r) const;
	virtual void		_draw_menupan_checked_item_check(NVGcontext* vg, const glm::rect& r,bool under)  const;
	virtual void		_path_button_box(NVGcontext* vg, const glm::rect& r, int boxType)  const;
	virtual void		_draw_button_box_over(NVGcontext* vg, const glm::rect& r, int state, int boxType) const;
	virtual void		_draw_button_box_image(NVGcontext* vg, const glm::rect& r, const NVGpaint& paint, int state, int boxType)  const;
	virtual void		_draw_button_box(NVGcontext* vg, const glm::rect& r, int state, int boxType)  const;
	virtual void		_draw_splitter_separator_horz(NVGcontext* vg, const glm::rect& r, float x )  const;
	virtual void		_draw_splitter_separator_vert(NVGcontext* vg, const glm::rect& r, float y )  const;
	virtual void		_draw_popupbox_box(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_menupan_box(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_menupan_separator(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_menupan_box_over(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_toolbar_box_base(NVGcontext* vg, const glm::rect& r)  const;
	virtual void		_draw_toolbar_box(NVGcontext* vg, const glm::rect& r, int s)  const;
	virtual void		_draw_statusbar_box(NVGcontext* vg, const glm::rect& r, int s) const;
	virtual void		_draw_options_control(NVGcontext* vg, const glm::rect& r, int state ) const;
	virtual void		_draw_options_box(NVGcontext* vg, const glm::rect& r, int state )  const;
	virtual void		_draw_options_text(NVGcontext* vg, const glm::rect& r, const std::string& str, int state )  const;
	virtual void		_draw_check_box(NVGcontext* vg, const glm::rect& r, int state )  const;
	virtual void		_draw_check_checked(NVGcontext* vg, const glm::rect& r, int state )  const;

	virtual float		_size_options_button_width				(const dim2& r)const;
	virtual float		_size_options_arrow_hwidth				(const dim2& r)const;
	virtual float		_size_options_arrow_y_offset			(const dim2& r)const;
	virtual float		_size_options_arrow_height				(const dim2& r)const;
	
	
	virtual NVGcolor	_color_tooltip_box						()const;
	virtual NVGcolor	_color_tooltip_bound					()const;
	virtual float		_width_tooltip_bound					()const;
	virtual fontname	_font_tooltip_text						()const;
	virtual float		_pt_tooltip_text						()const;
	virtual NVGcolor	_color_tooltip_text						()const;
	virtual float		_radius_tooltip_box						()const;
	virtual float		_size_tooltip_padding					()const;
	virtual dim2		_measure_tooltip_text(NVGcontext* vg, const string& s)const;
	virtual void		_draw_tooltip(NVGcontext* vg, const glm::rect& p, float dx, const string& str, float alpha )  const;
	virtual vec2		_vec_tooltip_offset						()const;
	
	
	virtual void		_draw_textinputbase_box(NVGcontext* vg, const rect& r, int state);
	virtual void		_draw_textinputbase_selection(NVGcontext* vg, const rect& r, const rect& sr, float x0, float x1, int state);
	virtual void		_draw_textinputbase_cursor(NVGcontext* vg, const rect& r, const rect& sr, float x, int state);
	virtual void		_draw_textinputbase_text(NVGcontext* vg, const rect& r, const rect& sr, const string& str, int align, int state);
	virtual dim2		_measure_textinputbase_text(NVGcontext* vg, const string& str);
	
	virtual float		_pos_slider_tick						(const rect& r) const;
	virtual float		_pos_slider_number						(int type, const rect& r) const;
	virtual float		_size_slider_ticks_minor				() const;
	virtual float		_size_slider_ticks_major				() const;
	virtual float		_size_slider_ticks_fine					() const;
	virtual float		_size_slider_padding					() const;
	virtual float		_size_slider_bar_height					() const;
	virtual float		_size_slider_tick_minWidth				() const;
	virtual float		_radius_f_slider_cursor					(int type)const;
	virtual float		_pos_f_slider_y							(int type,const rect&r)const;
	virtual void		_draw_slider_cursor						(NVGcontext* vg,int type,const rect& r,float x,int s)const;
	virtual void		_draw_slider_ticks_minor				(NVGcontext* vg,int type,const rect& r,int s,float offset,float step)const;
	virtual void		_draw_slider_ticks_major				(NVGcontext* vg,int type,const rect& r,int s,float offset,float step)const;
	virtual void		_draw_slider_ticks_fine					(NVGcontext* vg,int type,const rect& r,int s,float offset,float step)const;
	virtual void		_draw_slider_numbers					(NVGcontext* vg,int type,const rect& r,int s,float offset,float step, float valueOffset,float valueStep,double base)const;
	virtual void		_draw_slider_bar						(NVGcontext* vg,int type,const rect& r,int s) const;
};

inline float _size_property_label_offset() {
	return Theme::getCurrentTheme()->_size_property_label_offset();
}
inline NVGcolor _color_label() {
	return Theme::getCurrentTheme()->_color_label();
}
inline NVGcolor _color_targetted_glow() {
	return Theme::getCurrentTheme()->_color_targetted_glow();
}
inline NVGcolor _color_menu_text_under_item() {
	return Theme::getCurrentTheme()->_color_menupan_fill_under_item();
}
inline NVGcolor _color_f_label(int s) {
	return Theme::getCurrentTheme()->_color_f_label(s);
}
inline NVGcolor _color_f_button_fill(int s) {
	return Theme::getCurrentTheme()->_color_f_button_fill(s);
}
inline NVGcolor _color_f_button_bound(int s) {
	return Theme::getCurrentTheme()->_color_f_button_bound(s);
}
inline NVGcolor _color_button_shade_pushed() {
	return Theme::getCurrentTheme()->_color_button_shade_pushed();
}

inline fontname _font_menuitem() {
	return Theme::getCurrentTheme()->_font_menuitem();
}
inline float _pt_label() {
	return Theme::getCurrentTheme()->_pt_label();
}
inline void _draw_statusbar_box(NVGcontext* vg, const glm::rect& r, int s ) {
	Theme::getCurrentTheme()->_draw_statusbar_box(vg, r, s);
}
inline void _draw_menuitem(NVGcontext* vg, const glm::rect& r, const std::string& l, int align, int state, bool disabled, bool isSeparator ) {
	Theme::getCurrentTheme()->_draw_menuitem(vg, r, l, align, state, disabled, isSeparator);
}
inline glm::dim2 _measure_menuitem(NVGcontext* vg, const std::string& l, bool isSeparator ) {
	return Theme::getCurrentTheme()->_measure_menuitem(vg, l, isSeparator);
}
inline void _draw_menupan_box(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_menupan_box(vg, r);
}
inline void _draw_menupan_box_over(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_menupan_box_over(vg, r);
}

inline void _draw_menupan_under_item_fill(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_menupan_under_item_fill(vg, r);
}
inline void _draw_menupan_checked_item_check(NVGcontext* vg, const glm::rect& r, bool under) {
	Theme::getCurrentTheme()->_draw_menupan_checked_item_check(vg, r, under);
}
inline void _draw_menupan_separator(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_menupan_separator(vg, r);
}
inline void _draw_imagelabel(NVGcontext* vg, const glm::rect& r, const NVGpaint& p, int s) {
	Theme::getCurrentTheme()->_draw_imagelabel(vg, r, p, s);
}
inline glm::dim2 _measure_text_label(NVGcontext* vg, const std::string& l, float pt ) {
	return Theme::getCurrentTheme()->_measure_text_label(vg, l, pt);
}
inline void _draw_text_label(NVGcontext* vg, const glm::rect& r, const NVGcolor& c, const std::string& l, float pt, int a, int state) {
	Theme::getCurrentTheme()->_draw_text_label(vg, r, c, l, pt, a, state);
}



inline float _size_options_button_width(const dim2& r) {
	return Theme::getCurrentTheme()->_size_options_button_width(r);
}
inline float _size_options_arrow_y_offset(const dim2& r) {
	return Theme::getCurrentTheme()->_size_options_arrow_y_offset(r);
}
inline float _size_options_arrow_height(const dim2& r) {
	return Theme::getCurrentTheme()->_size_options_arrow_height(r);
}
inline float _size_options_arrow_hwidth(const dim2& r) {
	return Theme::getCurrentTheme()->_size_options_arrow_hwidth(r);
}


inline float _size_menupan_menuitem_height() {
	return Theme::getCurrentTheme()->_size_menupan_menuitem_height();
}
inline float _pt_menu_text() {
	return Theme::getCurrentTheme()->_pt_menu_text();
}
inline float _size_options_box_horz_padding() {
	return Theme::getCurrentTheme()->_size_options_box_horz_padding();
}
inline NVGcolor _color_panel() {
	return Theme::getCurrentTheme()->_color_panel();
}
inline NVGcolor _color_background() {
	return Theme::getCurrentTheme()->_color_background();
}
inline float _size_button_height() {
	return Theme::getCurrentTheme()->_size_button_height();
}
inline float _size_button_width() {
	return Theme::getCurrentTheme()->_size_button_width();
}
inline float _size_toolbar_height() {
	return Theme::getCurrentTheme()->_size_toolbar_height();
}
inline float _size_statusbar_height() {
	return Theme::getCurrentTheme()->_size_statusbar_height();
}
inline float _size_split_min() {
	return Theme::getCurrentTheme()->_size_split_min();
}
inline float _size_scroller_offset() {
	return Theme::getCurrentTheme()->_size_scroller_offset();
}
inline float _size_splitter_spacing() {
	return Theme::getCurrentTheme()->_size_splitter_spacing();
}
inline float _size_menupan_padding_horz() {
	return Theme::getCurrentTheme()->_size_menupan_padding_horz();
}
inline float _size_menupan_padding_vert() {
	return Theme::getCurrentTheme()->_size_menupan_padding_vert();
}
inline bool _test_scrollbar_horz(const glm::rect& r, const glm::vec2& pt) {
	return Theme::getCurrentTheme()->_test_scrollbar_horz(r,pt);
}
inline bool _test_scrollbar_vert(const glm::rect& r, const glm::vec2& pt) {
	return Theme::getCurrentTheme()->_test_scrollbar_vert(r,pt);
}


inline dim2 _dim_options_action_min_size() {
	return Theme::getCurrentTheme()->_dim_options_action_min_size();
}
inline dim2 _dim_check_action_size() {
	return Theme::getCurrentTheme()->_dim_check_action_size();
}






inline void _draw_popupbox_box(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_popupbox_box(vg, r);
}
inline void _draw_splitter_separator_horz(NVGcontext* vg, const glm::rect& r, float x) {
	Theme::getCurrentTheme()->_draw_splitter_separator_horz(vg, r, x);
}
inline void _draw_splitter_separator_vert(NVGcontext* vg, const glm::rect& r, float y) {
	Theme::getCurrentTheme()->_draw_splitter_separator_vert(vg, r, y);
}
inline void _draw_scroll_indicator_left(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_scroll_indicator_left(vg, r);
}
inline void _draw_scroll_indicator_right(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_scroll_indicator_right(vg, r);
}
inline void _draw_scroll_indicator_top(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_scroll_indicator_top(vg, r);
}
inline void _draw_scroll_indicator_bottom(NVGcontext* vg, const glm::rect& r) {
	Theme::getCurrentTheme()->_draw_scroll_indicator_bottom(vg, r);
}
inline void _draw_scrollbar_horz(NVGcontext* vg, const glm::rect& r, float alpha, float sz, float total, float am) {
	Theme::getCurrentTheme()->_draw_scrollbar_horz(vg, r, alpha, sz, total, am);
}
inline void _draw_scrollbar_vert(NVGcontext* vg, const glm::rect& r, float alpha, float sz, float total, float am) {
	Theme::getCurrentTheme()->_draw_scrollbar_vert(vg, r, alpha, sz, total, am);
}



inline void _draw_toolbar_box(NVGcontext* vg, const glm::rect& r, int s) {
	Theme::getCurrentTheme()->_draw_toolbar_box(vg, r, s);
}
inline void _draw_button_box(NVGcontext* vg, const glm::rect& r, int s, int t) {
	Theme::getCurrentTheme()->_draw_button_box(vg, r, s, t);
}
inline void _draw_button_box_over(NVGcontext* vg, const glm::rect& r, int s, int t) {
	Theme::getCurrentTheme()->_draw_button_box_over(vg, r, s, t);
}
inline void _draw_button_box_image(NVGcontext* vg, const glm::rect& r, const NVGpaint& p, int s, int t) {
	Theme::getCurrentTheme()->_draw_button_box_image(vg, r, p, s, t);
}






inline float _size_tooltip_padding() {
	return Theme::getCurrentTheme()->_size_tooltip_padding();
}

inline void _draw_options_box(NVGcontext* vg, const glm::rect& r, int s) {
	Theme::getCurrentTheme()->_draw_options_box(vg, r, s);
}
inline void _draw_options_text(NVGcontext* vg, const glm::rect& r, const std::string& l, int s) {
	Theme::getCurrentTheme()->_draw_options_text(vg, r, l, s);
}
inline void _draw_options_control(NVGcontext* vg, const glm::rect& r, int s) {
	Theme::getCurrentTheme()->_draw_options_control(vg, r, s);
}
inline void _draw_check_box(NVGcontext* vg, const glm::rect& r, int s) {
	Theme::getCurrentTheme()->_draw_check_box(vg, r, s);
}
inline void _draw_check_checked(NVGcontext* vg, const glm::rect& r, int s) {
	Theme::getCurrentTheme()->_draw_check_checked(vg, r, s);
}
inline void _draw_tooltip(NVGcontext* vg, const glm::rect& r, float dx, const string& str, float alpha ) {
	Theme::getCurrentTheme()->_draw_tooltip(vg, r, dx, str, alpha);
}
inline dim2	_measure_tooltip_text(NVGcontext* vg,  const string& s) {
	return Theme::getCurrentTheme()->_measure_tooltip_text(vg, s);
}

inline vec2	_vec_tooltip_offset() {
	return Theme::getCurrentTheme()->_vec_tooltip_offset();
}


inline void _draw_textinputbase_box(NVGcontext* vg, const rect& r, int s) {
	Theme::getCurrentTheme()->_draw_textinputbase_box(vg,r,s);
}
inline void _draw_textinputbase_selection(NVGcontext* vg, const rect& r, const rect& sr, float x0, float x1, int s) {
	Theme::getCurrentTheme()->_draw_textinputbase_selection(vg,r,sr,x0,x1,s);
}
inline void _draw_textinputbase_cursor(NVGcontext* vg, const rect& r, const rect& sr, float x, int s) {
	Theme::getCurrentTheme()->_draw_textinputbase_cursor(vg,r,sr,x,s);
}
inline void _draw_textinputbase_text(NVGcontext* vg, const rect& r, const rect& sr, const string& str, int align, int s) {
	Theme::getCurrentTheme()->_draw_textinputbase_text(vg,r,sr,str,align,s);
}
inline dim2 _measure_textinputbase_text(NVGcontext* vg, const string& str) {
	return Theme::getCurrentTheme()->_measure_textinputbase_text(vg,str);
}




inline float _pos_slider_tick (const rect& r) {
	return Theme::getCurrentTheme()->_pos_slider_tick(r);
}
inline float _size_slider_padding () {
	return Theme::getCurrentTheme()->_size_slider_padding();
}
inline float	_radius_f_slider_cursor(int type) {
	return Theme::getCurrentTheme()->_radius_f_slider_cursor(type);
}
inline float	_pos_f_slider_y(int type,const rect&r) {
	return Theme::getCurrentTheme()->_pos_f_slider_y(type,r);
}
inline float	_pos_slider_number(int type,const rect&r) {
	return Theme::getCurrentTheme()->_pos_slider_number(type,r);
}



inline void		_draw_slider_cursor(NVGcontext* vg,int type,const rect& r,float x,int s) {
	return Theme::getCurrentTheme()->_draw_slider_cursor(vg,type,r,x,s);
}
inline void		_draw_slider_ticks_major(NVGcontext* vg,int type,const rect& r,int s,float offset,float step) {
	return Theme::getCurrentTheme()->_draw_slider_ticks_major(vg,type,r,s,offset,step);
}
inline void		_draw_slider_ticks_minor(NVGcontext* vg,int type,const rect& r,int s,float offset,float step) {
	return Theme::getCurrentTheme()->_draw_slider_ticks_minor(vg,type,r,s,offset,step);
}
inline void		_draw_slider_ticks_fine(NVGcontext* vg,int type,const rect& r,int s,float offset,float step) {
	return Theme::getCurrentTheme()->_draw_slider_ticks_fine(vg,type,r,s,offset,step);
}
inline void		_draw_slider_numbers(NVGcontext* vg,int type,const rect& r,int s,float offset,float step, float valueOffset,float valueStep,double base) {
	return Theme::getCurrentTheme()->_draw_slider_numbers(vg,type,r,s,offset,step,valueOffset,valueStep,base);
}
inline void _draw_slider_bar(NVGcontext* vg,int type,const rect& r,int s)  {
	return Theme::getCurrentTheme()->_draw_slider_bar(vg,type,r,s);
}


inline NVGcolor _color_tooltip_box() {
	return Theme::getCurrentTheme()->_color_tooltip_box();
}
inline NVGcolor _color_tooltip_bound() {
	return Theme::getCurrentTheme()->_color_tooltip_bound();
}
inline NVGcolor _color_tooltip_text() {
	return Theme::getCurrentTheme()->_color_tooltip_text();
}
inline float _radius_tooltip_box() {
	return Theme::getCurrentTheme()->_radius_tooltip_box();
}
inline float _width_tooltip_bound() {
	return Theme::getCurrentTheme()->_width_tooltip_bound();
}
inline float _pt_tooltip_text() {
	return Theme::getCurrentTheme()->_pt_tooltip_text();
}
inline const char* _font_tooltip_text() {
	return Theme::getCurrentTheme()->_font_tooltip_text();
}


} // namespace JGL

#endif /* JGL__Theme_hpp */
