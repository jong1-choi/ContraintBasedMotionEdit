//
//  JGL.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/17.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_h
#define JGL_h

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <opengl/gl3.h>
#else
#define GLEW_STATIC
#include <gl/glew.h>
#pragma comment (lib,"glew32s")
#pragma comment (lib,"opengl32.lib")
#pragma comment (lib,"glfw3.lib")
#endif


#include <vector>
#include <stack>
#include <string>
#ifdef __APPLE__
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "nanovg/nanovg.h"

#include "JGL__Math.hpp"
#include "JGL__String.hpp"

namespace JGL {

using namespace glm;
using namespace std;

struct Widget;
struct Window;
struct Group;
struct _Popup;
struct _PopupHandler;

enum {
	EVENT_MOVE				= 10,
	EVENT_DRAG				= 11,
	EVENT_PUSH				= 12,
	EVENT_RELEASE			= 13,
	EVENT_CANCEL			= 15,
	EVENT_KEYDOWN			= 20,
	EVENT_KEYUP				= 21,
	EVENT_CHAR				= 22,
	EVENT_REPEAT			= 23,
	EVENT_ENTER				= 30,
	EVENT_LEAVE				= 31,
	EVENT_SCROLL			= 40,
	EVENT_ZOOM				= 41,
	EVENT_TARGETTED			= 50,
	EVENT_UNTARGETTED		= 51,
	EVENT_PASTE				= 60,
	EVENT_COPY				= 61,
	EVENT_CUT				= 62,
	EVENT_UNDO				= 64,
	EVENT_REDO				= 65,
	EVENT_DND				= 90,
	EVENT_POPUP_DISMISSED	= 100,
	EVENT_WINDOW_FOCUSED	= 110,
	EVENT_WINDOW_UNFOCUSED	= 111,
};

enum {
	EVENT_LBUTTON 			= (1<<0),
	EVENT_RBUTTON 			= (1<<1),
	EVENT_MBUTTON 			= (1<<2),
	EVENT_XBUTTON 			= (1<<3),
	EVENT_YBUTTON 			= (1<<4),
};

enum {
	CURSOR_ARROW		= GLFW_ARROW_CURSOR,
	CURSOR_IBEAM		= GLFW_IBEAM_CURSOR,
	CURSOR_CROSSHAIR	= GLFW_CROSSHAIR_CURSOR,
	CURSOR_HAND			= GLFW_HAND_CURSOR,
	CURSOR_HRESIZE		= GLFW_HRESIZE_CURSOR,
	CURSOR_VRESIZE		= GLFW_VRESIZE_CURSOR,
};

enum {
	MOD_LSHIFT			= (1<<0),
	MOD_RSHIFT			= (1<<1),
	MOD_SHIFT			= (MOD_LSHIFT|MOD_RSHIFT),
	MOD_LCONTROL		= (1<<2),
	MOD_RCONTROL		= (1<<3),
	MOD_CONTROL			= (MOD_LCONTROL|MOD_RCONTROL),
	MOD_LALT			= (1<<4),
	MOD_RALT			= (1<<5),
	MOD_ALT				= (MOD_LALT|MOD_RALT),
	MOD_LSUPER			= (1<<6),
	MOD_RSUPER			= (1<<7),
	MOD_SUPER			= (MOD_LSUPER|MOD_RSUPER),
	MOD_LBUTTON			= (1<<8),
	MOD_RBUTTON			= (1<<9),
	MOD_MBUTTON			= (1<<10),
	MOD_XBUTTON			= (1<<11),
	MOD_YBUTTON			= (1<<12),
};

enum {
	SHADE_WINDOW_UNFOCUSED		=(1<<0),
	SHADE_INACTIVE				=(1<<1),
	SHADE_TARGETTED				=(1<<2),
	SHADE_PRESSED				=(1<<3),
	SHADE_ENGAGED				=(1<<4),
};

enum {
	REASON_NONE					=0,
	REASON_OTHER_START			=(1<<5),
	REASON_ESC					=(1<<3),
	REASON_WINDOW_UNFOCUSED		=(1<<4),

	REASON_CLICK_OUTSIDE		=(1<<0),
//	REASON_RELEASE_INSIDE		=(1<<1),
	REASON_POPUP_ACTION			=(1<<2),

	REASON_OTHER_DISMISS		=(1<<6),
	REASON_CHILD_START			=(1<<7),
	REASON_CHILD_ACTION			=(1<<8),
	REASON_CHILD_CANCEL			=(1<<9),

	REASON_TYPE_CASCADE_MENU	=REASON_CLICK_OUTSIDE
								|REASON_POPUP_ACTION
								|REASON_OTHER_START
								|REASON_CHILD_ACTION
								|REASON_WINDOW_UNFOCUSED,
	REASON_TYPE_DIALOG			=REASON_POPUP_ACTION|REASON_ESC,
	REASON_ALL					=(1<<10)-1,
};

enum {
	ALIGN_NONE			= 0,
	ALIGN_TOP			= (1<<0),
	ALIGN_BOTTOM		= (1<<1),
	ALIGN_LEFT			= (1<<2),
	ALIGN_RIGHT			= (1<<3),
	ALIGN_SIDE			= ALIGN_LEFT|ALIGN_RIGHT,
	ALIGN_TOP_BOTTOM	= ALIGN_TOP|ALIGN_BOTTOM,
	ALIGN_ALL			= ALIGN_SIDE|ALIGN_TOP_BOTTOM,
};

enum {
	AUTOSCALE_NONE		= 0,
	AUTOSCALE_W			= 1,
	AUTOSCALE_H			= 2,
	AUTOSCALE_ALL		= 3,
};

enum {
	LABEL_NONE			= 0,
	LABEL_LEFT			= 1,
	LABEL_RIGHT			= 2,
	LABEL_BOTTOM		= 3,
	LABEL_TOP			= 4,
};

enum {
	BUTTON_BOX_IND = 0,
	BUTTON_BOX_LEFT = 1,
	BUTTON_BOX_RIGHT = 2,
	BUTTON_BOX_TOP = 3,
	BUTTON_BOX_BOTTOM = 4,
	BUTTON_BOX_MIDDLE = 5,
};

struct _JGL {
	
	//* Adding group management
	static void		pushAddingGroup(Group*);
	static void		clearAddingGroup();
	static void		popAddingGroup();
	static void		popAddingGroupUntil( Group* );
	static Group*	getCurrentAddingGroup();
	static void		addToCurrentGroup( Widget* );

	//* Event context (for which Widget the event is being dispatched.)
	static const Widget*	eventContext();
	static void		clearEventContext();
	
	//* Event dispatch function
	static bool		dispatchEvent( Widget* w, int event );

	//* Event data retrival functions
	static float				eventX();
	static float				eventY();
	static vec2					eventPt();
	static float				eventScrollX();
	static float				eventScrollY();
	static vec2					eventScroll();
	static float				eventZoom();
	static vector<string>const& eventDND();
	static float				eventWindowX();
	static float				eventWindowY();
	static vec2					eventWindowPt();
	static bool					eventMods( int mod );
	static int					eventKey();
	static int					eventButton();
	static char32_t				eventCodePoint();
	static const Window*		eventWindow();
	static NVGcontext*			eventWindowContext();
	static const string&		eventPasteString();
	static void					eventCopyString( const string& );


	//* Popup management
	static bool				startPopup( Widget* w, Widget* parent, const vec2& p, int r=REASON_ALL );
	static void				dismissPopup( _PopupHandler* win, _Popup* p, int reason );
	static void				dismissPopups( _PopupHandler* win, int reason );
		// Widget leven request..
	static void		dismissPopup(Widget* w, int reason);

	//* Run
	static void		run();
	
	
	//* Window drawing management
	static void				setCurrentDrawWindow( Window* win );
	static const Window*	currentDrawWindow();
	static NVGcontext*		getCurrentNVGContext();
	static bool				getCurrentDrawWindowFocused();
	static float			getCurrentDrawWindowPxRatio();

	//* A function to be called for drawing a child (or member) widget
	static void				drawAsChild( NVGcontext* vg, Widget* w );
	
	//* Window level control functions
	static void				setCursor( int cursor );
	
protected:
	
	// Window management variables + focussed window
	static std::vector<Window*> _windows;
	static int _focusedWindow;
	
	// Event data storage
	static glm::vec2 _mousePt;
	static glm::vec2 _mouseLButtonPressedPt;
	static glm::vec2 _mouseRButtonPressedPt;
	static glm::vec2 _mouseMButtonPressedPt;
	static glm::vec2 _mouseXButtonPressedPt;
	static glm::vec2 _mouseYButtonPressedPt;

	static Widget* _mouseLButtonPressedWidget;
	static Widget* _mouseRButtonPressedWidget;
	static Widget* _mouseMButtonPressedWidget;
	static Widget* _mouseXButtonPressedWidget;
	static Widget* _mouseYButtonPressedWidget;

	static bool		_pressForFocus;
	static int 		_pressedButtons;
	static vec2		_scrollDelta;
	static float	_zoomFactor;
	static int   	_modState;
	static int   	_eventKey;
	static int  	_eventButton;
	static char32_t _eventCodePoint;
	static string	_eventPasteString;
	static vector<string> _eventDropedStrings;

	
	// Adding group
	static vector<Group*> _currentAddingGroup;

	// Event context related function
	static Widget* _currentEventContext;
	static Widget* _currentUnderWidget;
	static Widget* _currentTargetWidget;

	static Window* _currentDrawWindow;
	static Window* _currentEventWindow;

	//* Very special function to dispatch "ENTER/LEAVE" events
	static void		propagateLeaveEnter( Widget* w, const glm::vec2& oldPt, const glm::vec2& newPt );

	
	//* Window management functions
	static void			registerWindow( Window* win );
	static void			registerWindowCallbacks( Window* win );
	static GLFWwindow* 	getShaderableContext();

	
	static Window*		eventWindow_();
	static int			searchWindow( GLFWwindow* window );
	static void			mousePositionCallback( GLFWwindow* window, double x, double y);
	static void			mouseButtonCallback( GLFWwindow* window, int button, int action, int mods );
	static void			keyCallback( GLFWwindow* window, int key, int scan, int action, int mods );
	static void			charCallback( GLFWwindow* window, unsigned int codepoint);
	static void			resizeCallback( GLFWwindow* window, int w, int h );
	static void			fbResizeCallback( GLFWwindow* window, int w, int h );
	static void			windowFocusCallback( GLFWwindow* window, int );
	static void			scrollCallback( GLFWwindow* window,  double, double );
	static void			zoomCallback( GLFWwindow* window,  double );
	static void			dragAndDropCallback( GLFWwindow* win, int n, const char*[] );
	
	static void			eventContext( Widget* g );

	friend Window;
};

} // namespace JGL

#endif /* JGL_h */
