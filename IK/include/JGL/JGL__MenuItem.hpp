//
//  JGL__MenuItem.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/27.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__MenuItem_hpp
#define JGL__MenuItem_hpp

#include "JGL_Widget.hpp"

namespace JGL {

struct _MenuItem {
	_MenuItem(const std::string& label, int data, bool disabled=false);
	
	virtual dim2				measure(NVGcontext* vg);
	virtual void				draw(NVGcontext* vg, const glm::rect& r, int align, int state);
		
	virtual bool				disabled() const { return _disabled; }
	virtual void				enable() { _disabled=false; }
	virtual void				disable() { _disabled=true; }
	virtual bool				isSeparator() { return _label.compare("----")==0; }
	
	// Submenu control functions
	virtual void				add( const _MenuItem& item );
	virtual void				add( const std::string& name, int data, bool disabled=false );
	virtual void				add( const std::vector<_MenuItem>& items );
	virtual void				clear();
	virtual void				disable(int idx);
	virtual void				enable(int idx);
	virtual _MenuItem&			item(int idx);
	virtual const _MenuItem&	item(int idx) const;
	
	virtual string&				label() { return _label; }
	virtual const string&		label() const { return _label; }
	virtual void				label(const string& s) { _label=s; }
	virtual int					data() { return _data; }
	virtual const int&			data() const { return _data; }
	virtual void				data(int d) { _data=d; }
protected:
	std::string					_label = "----";
	int							_data = 0;
	bool						_disabled = false;
	std::vector<_MenuItem>		_items;
};

} // namespace JGL

#endif /* JGL__MenuItem_hpp */
