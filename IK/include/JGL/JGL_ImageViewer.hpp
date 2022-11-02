//
//  JGL_ImageViewer.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/18.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_ImageViewer_hpp
#define JGL_ImageViewer_hpp

#include "JGL_Widget.hpp"
#include "JGL__Draw.hpp"
#include "JGL__Scrollable.hpp"
#include "JGL__Targettable.hpp"

namespace JGL {

struct TexImage {
	// The size of the current texture
	unsigned int _texWidth  = 0;
	unsigned int _texHeight = 0;
	GLuint _texID = 0;
	
	// The size of the image requested to setup
	unsigned int _imageWidth = 0;
	unsigned int _imageHeight = 0;
	unsigned char* _data = nullptr;
	bool _BGR = true;
	bool _changed = false;

	virtual GLuint texId() const { return _texID; }
	virtual void makeGLImage();
	virtual void clear();
	virtual void setImage( const unsigned char* d, int ww, int hh, bool isBGR=true );
	virtual bool changed() const { return _changed; }
	idim2 texSize() const { return idim2(_texWidth,_texHeight); }
	idim2 imgSize() const { return idim2(_imageWidth,_imageHeight); }
	
	unsigned char* createThumbnail( int& iw, int& ih, int THUMB_W=256, int THUMB_H=256 );

	~TexImage();
	TexImage( TexImage&& a );
	TexImage();
};



struct ImageViewer: public Widget, public _Targettable {

	ImageViewer(float xx, float yy, float ww, float hh, const string& title="" );
	
	virtual void clear();
	virtual void setImage( const unsigned char* d, int ww, int hh, bool isBGR=true );

	virtual void resizableToImage( bool resizable ) { _resizableToImage = resizable; }
	virtual bool resizableToImage() const { return _resizableToImage; }
	virtual TexImage& texImage() { return _texImage; }
	virtual const TexImage& texImage() const { return _texImage; }
	
	void fitToScreen();
	
	template<typename T>
	glm::vec<2,T> screenToImage( const glm::vec<2,T>& pt ) const;
	template<typename T>
	glm::vec<2,T>		imageToScreen( const glm::vec<2,T>& pt ) const;
	template<typename T>
	glm::vec<2,T> screenToImage( const glm::vec<2,T>& pt, const glm::vec2& offset ) const;
	template<typename T>
	glm::vec<2,T>		imageToScreen( const glm::vec<2,T>& pt, const glm::vec2& offset ) const;
	virtual rect		getScreenRect( const glm::rect& rect, const glm::vec2& offset ) const;
	virtual irect 		getScreenRect( const glm::irect& rect, const glm::vec2& offset ) const;
	virtual rect		getScreenRect( const glm::rect& rect ) const;
	virtual irect 		getScreenRect( const glm::irect& rect ) const;
	virtual bool 		zoomAction( const vec2& mousePt, float zoomDelta );

protected:
	
	virtual void	rearrange(NVGcontext* vg,int scaling) override;
	virtual void	drawImage();
	
	virtual void	drawBoxOver(NVGcontext* vg,const rect&r) override;
	virtual void	drawContents(NVGcontext* vg,const rect&r,int a) override{};

	virtual void	drawGL() override;
	virtual bool	handle( int e ) override;

	virtual void	updateFitScale();

	float _minWidth = 100.f;
	float _maxWidth = 1920.f;
	float _minHeight= 100.f;
	float _maxHeight= 1300.f;

	bool _resizableToImage = false;
	
	TexImage _texImage;
	_Scrollable _scroller;
	vec2 _lastPt;
	
	float _viewScale = 1.f;	// image -> window: imageSize* scale = windowSpan
	float _desiredScale = 1.f;
	float _fitScale = 1.f;

	bool _fitted = false;
	bool _keepFitted = false;

};


template<typename T> inline glm::vec<2,T> ImageViewer::screenToImage( const glm::vec<2,T>& pt, const glm::vec2& offset ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x+offset.x)/_viewScale);
	pti.y = T((pt.y+offset.y)/_viewScale);
	return pti;
}
template<typename T> inline glm::vec<2,T> ImageViewer::screenToImage( const glm::vec<2,T>& pt ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x+_scroller.scrollOffset().x)/_viewScale);
	pti.y = T((pt.y+_scroller.scrollOffset().y)/_viewScale);
	return pti;
}

template<typename T> inline glm::vec<2,T> ImageViewer::imageToScreen( const glm::vec<2,T>& pt, const glm::vec2& offset ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x)*_viewScale-offset.x);
	pti.y = T((pt.y)*_viewScale-offset.y);
	return pti;
}

template<typename T> inline glm::vec<2,T> ImageViewer::imageToScreen( const glm::vec<2,T>& pt ) const {
	glm::vec<2,T> pti;
	pti.x = T((pt.x)*_viewScale-_scroller.scrollOffset().x);
	pti.y = T((pt.y)*_viewScale-_scroller.scrollOffset().y);
	return pti;
}


} // namespace JGL

#endif /* JGL_ImageViewer_hpp */
