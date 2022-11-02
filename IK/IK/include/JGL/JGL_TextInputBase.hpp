//
//  JGL_TextInputBase.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/03.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_TextInputBase_hpp
#define JGL_TextInputBase_hpp


#include "JGL_Property.hpp"
#include "JGL__Targettable.hpp"

namespace JGL {

enum {
	OP_SET,
	OP_EDIT,
	OP_PASTE,
	OP_CUT,
	OP_OTHER,
};


struct TextInputBase: public Property, public _Targettable {
	TextInputBase(float x,float y,float w,float h, const std::string& label );

	virtual bool		textDamaged() { return _textDamaged; }
	virtual void		textDamage();

	virtual void		textAlignment(int a);
	virtual int 		textAlignment() const { return _textAlignment; }
	
	virtual void		select(int start,int end);
	virtual void		deleteSelection();
	virtual void		append( const string& s);
	virtual void		replace( const string& s);
	
	virtual bool		handle(int event) override;
	
	virtual void		enterPressed();
	virtual void		enterPressedCallback( Callback c, void* ud=nullptr);
	
	virtual string		str() const { return _str; }
	virtual void		str(const string& s);
	virtual void		stringEntered(){}
	
protected:

	virtual bool		valid(const string& s) const =0;

	virtual float		boxHorzPadding() const { return _boxHorzPadding<0?_size_options_box_horz_padding():_boxHorzPadding; }
	virtual dim2		minActionSize(NVGcontext*vg) const override;
	virtual void		drawAction(NVGcontext* vg, const rect& r, int alignment) override;

	virtual void		updateLocations(NVGcontext*vg);
	virtual void		stringChanged();

	virtual float		cursorLoc(int c);
	virtual float		cursorPos(int c);
	virtual int			ptToCursor( const vec2& pt );
	virtual void		updateScroll(const rect& r);

	string				_str ="";
	int					_cursor = 0;
	int					_selectionL=0, _selectionR=0;
	vector<float>		_locations;
	int					_textAlignment = ALIGN_LEFT;
	float				_boxHorzPadding = -1.f;
	float				_scrolled = 0;
	vec2				_textOffset = vec2(0,0);
	dim2				_textSz = dim2(0,0);
	bool				_textDamaged = false;
	int					_clickedCursor = 0;		
	
	Callback			_enterPressedCallback = nullptr;
	void*				_enterPressedUserdata = nullptr;


	// For undo...
	virtual bool		undo();
	virtual bool		redo();
	virtual void		storeUndo(int operation);
	virtual void		setUndoCursor();
	virtual void		clearLastOp();
	virtual void		clearUndo();
	vector<string>		_undoStack;
	vector<int>			_undoCursorStack;
	int					_undoValidPointer = -1;
	int					_undoPointer = -1;
	int					_lastOp = OP_OTHER;
	int					_maxHistory=5;
};

} // namespace JGL

#endif /* JGL_TextInputBase_hpp */
