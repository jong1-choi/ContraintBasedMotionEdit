//
//  JGL_MultiImageViewer.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/23.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL_MultiImageViewer_hpp
#define JGL_MultiImageViewer_hpp


#include "JGL_ImageViewer.hpp"
#include "JGL_RadioButtonGroup.hpp"

namespace JGL {

const float _def_image_selection_button_width = 36.f;
const float _def_image_selection_button_height = 24.f;
const float _def_image_selection_button_spacing = 6.f;
const float _def_image_selection_button_padding = 1.2f;

struct MultiImageViewer: public ImageViewer {
	
public:
	MultiImageViewer(float xx, float yy, float ww, float hh, const std::string& title=nullptr );
	
	virtual void	clear() override;
	virtual void	addImage( const unsigned char* d, int ww, int hh, bool isBGR=true );
	virtual void	setImage( const unsigned char* d, int ww, int hh, bool isBGR=true ) override;
	virtual TexImage& texImage() override;
	virtual void	visible( int x );

	virtual void	rearrange(NVGcontext* vg,int scaling) override;
protected:
	virtual void	drawContents(NVGcontext* vg,const rect&r, int a) override;
	virtual bool	handle(int event) override;
	static void		selectionCB( Widget* w, void* ud ) { ((MultiImageViewer*)ud)->selectionChanged(); }
	virtual void	selectionChanged();
	virtual void	drawGL() override;

	std::vector<TexImage> _texImages;
	std::vector<TexImage> _texImagesToDelete;
	int _visibleTexImage = 0;
	RadioButtonGroup _buttonGroup;
};

} // namespace JGL

#endif /* JGL_MultiImageViewer_hpp */
