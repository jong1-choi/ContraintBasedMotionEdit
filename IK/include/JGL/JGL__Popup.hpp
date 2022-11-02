//
//  JGL__Popup.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/27.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Popup_hpp
#define JGL__Popup_hpp

#include "JGL_Widget.hpp"

namespace JGL {

//* Special Widget wrapper to make it as a popup
//* It stores the information of popup
//*
//* Wrapped functions includes:
//*		bool changed() const;
//*		bool damaged() const;
//*		void rearrange();
//* Additional fields and functions are
//*		DismissReason _dismissReason=REASON_ALL;
//*			decide when the popup will be dismissed
//*
//*		Widget* _content = nullptr;
//*			The content widget, which can be virtually any Widget
//*			This is set by the contructor only for integraty
//*			That is why there is no setter and getter
//*
//*		Widget* _parent = nullptr;
//*			The parent Widget of the popup.
//*			The position of the popup will be computed based on the parent,
//*			and so does mouse position event.
//*			Moreover, "dismiss" will be propagate to the parent with
//*			the proper Reason.
//*
//*		bool    _mouseOnPopup = false;
//*			The flag corresponding to whether the mouse is on the popup
//*			This flag is used only for enter/leave event and auto dismissal.
//*
//*		bool	_mouseEverOnPopup = false;
//*			This flag is used only for auto dismissal.

struct _PopupHandler;
struct _Popup {
	
	_Popup(Widget* content=nullptr, Widget* parent=nullptr, int r=REASON_ALL);

	bool			changed() const;
	bool			damaged() const;
	void			reform(NVGcontext*vg, int scaling);
	void			size(const dim2& sz);
	bool			under() const;
	bool			under(const glm::vec2& pt) const;
	void			draw(NVGcontext* vg);

	void			dispatchEvent( int e );
	void			dismiss();
	
protected:
	
	Widget*			underWidget(const glm::vec2& pt) const;
	Widget*			contents() const { return _content; }

	Widget*	_content = nullptr;
	Widget* _parent = nullptr;
	bool	_mouseOnPopup = false;
	bool	_mouseEverOnPopup = false;
	int		_dismissReason=REASON_ALL;
	
	friend _PopupHandler;
	friend Window;
};

//* A supportive method set to handle (multiple/hierarchical) popups
//* This struct can be inherited by (virtual) any widget,
//* but in current implementaion of JGL::JGL (main runner),
//*  Window inherites PopupHandler and calls "draw(vg)" for it.
//* Also, JGL::JGL refers its list of "Windows" to find the right handler.
//*
//* The major functions of this handler are
//* 1. managing popup list
//*		std::vector<Popup> _popups;
//*			The real list of the popups
//*
//*		virtual bool hasPopup() const
//*			If the handler has one or more popups
//*
//*		virtual bool hasPopup(Popup* p) const
//*			If the handler has a particular popup
//*
//*		virtual Popup* matchingPopup(Widget* w)
//*			Find and return the popup that has the "w" as its "_content" if any.
//*
//*		virtual void addPopup( Popup p )
//*			Add new popup to the list.
//*			Of course only JGL::JGL should call it through "startPopup()"
//*
//*		virtual void removePopup( Popup* p )
//*			Remove the popup from the list.
//*			This function should be called only from "dismissPopup()"
//*
//*
//* Wrapped functions includes:
//*		virtual bool popupChanged() const;
//*		virtual bool popupDamaged() const;
//*		virtual void popupRearrange();
//*		virtual void popupDraw(NVGcontext*);
//*			Those functions calls the corresponding function of the
//*			content Widgets and collect the return values.
//*			with exception of "popupHandle()."
//*
//*		virtual bool popupHandle(int event) {
//*			"popupHandle()" actually handles events:
//*			a. check if the mouse is on the top most popup.
//*			b. if so, dispatch event to the popup
//*			c. produce enter/leave event
//*			d. detect the mouse click outside of the topmost widget for auto dismissal.
//*
//*		virtual void dismissPopup(Popup* p, DismissReason reason)
//*			The basic form of dismissal function.
//*			Try to dismiss the Popup with the given reason.
//*
//*			Internally, if there is sufficient reason, it dismiss the popup,
//*			and then, convert the reason to "child-" reason form to send
//*			to its parent, because some popups are affected (dismissed)
//*			by the child popup's dismissal.
//*
//*		virtual void dismissPopup(Widget* w, DismissReason reason)
//*			This variant is for "Widget" level dismissal.
//*			By sending "_content" widget as "w", the corresponding popup
//*			will be dismissed.
//*			This would be useful to dismiss a popup in "handle()" function.
//*
//*		virtual void dismissPopups(DismissReason reason)
//*			Try to dismiss each popup due to the window-wise event,
//*			such as "window-unfocused" or ESC key.
//*

struct _PopupHandler {
	
	virtual bool hasPopup() const;
	virtual bool hasPopup(_Popup* p) const;
	virtual const _Popup* matchingPopup(Widget* w) const;
	virtual _Popup* matchingPopup(Widget* w);

	virtual bool popupChanged() const;
	virtual void popupReform(NVGcontext* vg,int scaling);
	virtual void popupDraw(NVGcontext* vg);
	virtual bool popupHandle(int event);
	virtual Widget* popupUnderWidget(const glm::vec2& pt) const;

	virtual void dismissPopup(_Popup* p, int reason);
	virtual void dismissPopups(int reason);
	virtual void dismissPopup(Widget* w, int reason);

protected:
	virtual void addPopup( _Popup p );
	std::vector<_Popup> _popups;

private:
	virtual void removePopup( _Popup* p);
	
	friend _JGL;
};

} // namespace JGL

#endif /* JGL__Popup_hpp */
