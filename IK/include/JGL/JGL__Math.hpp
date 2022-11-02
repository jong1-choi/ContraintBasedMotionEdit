//
//  JGL__Math.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/09/29.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__Math_hpp
#define JGL__Math_hpp

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

namespace glm {

/// \brief 2D size
///
/// A structure (template) encapsulate 2-d size.
template<typename T> struct tdim2 {
	union {
		struct {
			T w,		///< the width
			h;			///< the height
		};
		struct {
			T p[2];		///< C array style access
		};
	};
	/// \brief Constructor.
	///
	/// @param ww The width of the size. Default is zero
	/// @param hh The height of the size. Default is zero
	inline				tdim2( const T& ww=0, const T& hh=0): h(hh), w(ww){}
	
	/// \brief Equality comparison operator.
	///
	/// @param a The dim2<T> to compare.
	/// @return True if the two sizes are the same.
	inline	bool		operator == (const tdim2& a) const { return w==a.w && h==a.h; }

	/// \brief Inequality comparison operator.
	///
	/// @param a The dim2<T> to compare.
	/// @return True if either the widths of the heights are not the same.
	inline	bool 		operator != (const tdim2& a) const { return w!=a.w || h!=a.h; }
	
	/// \brief Increment operator.
	///
	/// The width and height are increased simulataneously.
	/// Note that the positive-constraint is not enforced.
	/// @param a The dim2<T> to be added.
	/// @return The increased tdim2<T>.
	inline	tdim2&		operator += (const tdim2& a) { w+=a.w; h+=a.h; return *this; }

	/// \brief Decrement operator.
	///
	/// The width and height are decreased simulataneously.
	/// Note that the positive-constraint is not enforced.
	/// @param a The dim2<T> to be subtracted.
	/// @return The increased tdim2<T>.
	inline	tdim2&		operator -= (const tdim2& a) { w-=a.w; h-=a.h; return *this; }

	/// \brief Addition operator.
	///
	/// It returns the size increased by the parameter.
	/// Note that the positive-constraint is not enforced.
	/// @param a The dim2<T> to be added.
	/// @return The increased tdim2<T>.
	inline	tdim2		operator + (const tdim2& a) const { return tdim2(w+a.w,h+a.h); }

	/// \brief Subtraction operator.
	///
	/// It returns the size decreased by the parameter.
	/// Note that the positive-constraint is not enforced.
	/// @param a The dim2<T> to be subtracted.
	/// @return The decreased tdim2<T>.
	inline	tdim2		operator - (const tdim2& a) const { return tdim2(w-a.w,h-a.h); }

	/// \brief Multiplication operator.
	///
	/// It returns the size multiplied by the parameter.
	/// Note that the positive-constraint is not enforced.
	/// @param v The value to multiply with.
	/// @return The scaled tdim2<T>.
	inline	tdim2		operator * (T v) const { return tdim2(w*v,h*v); }

	/// \brief Division operator.
	///
	/// It returns the size divided by the parameter.
	/// Note that the positive-constraint is not enforced.
	/// @param v The value to divide with.
	/// @return The scaled tdim2<T>.
	inline	tdim2		operator / (T v) const { return tdim2(w/v,h/v); }

	/// \brief Multiplication operator.
	///
	/// It returns the vector consisted with the width and height individually
	/// multiplied by the parameter's x and y value.
	/// Note that the positive-constraint is not enforced.
	/// @param v The 2D vector to multiply with.
	/// @return The vector of which x is the scaled width, and y is the scaled height.
	inline	vec<2,T>	operator * (const vec<2,T>& v) const { return vec<2,T>(w*v.x,h*v.y); }

	/// \brief Division operator.
	///
	/// It returns the size consisted with the width and height individually
	/// divided by the parameter's x and y value.
	/// Note that the positive-constraint is not enforced.
	/// @param v The 2D vector to divided with.
	/// @return The vector of which x is the scaled width, and y is the scaled height.
	inline	vec<2,T>	operator / (const vec<2,T>& v) const { return vec<2,T>(w/v.x,h/v.y); }

	/// \brief Multiplication operator.
	///
	/// It returns the size multiplied by the parameter.
	/// Note that the positive-constraint is not enforced.
	/// @param v The value to multiply with.
	/// @param x The size to be multiply.
	/// @return The scaled tdim2<T>.
	friend	tdim2		operator * (T v, const tdim2& x) { return tdim2(x.w*v,x.h*v); }
	
	
	/// \brief Compute the maximum of two sizes.
	///
	/// It returns the size consisted with the maximum width and the maximum height among two tdim2<T>.
	/// Note that the positive-constraint is not enforced.
	/// @param a The first tdim2<T>.
	/// @param b The second tdim2<T>.
	/// @return The tdim2<T> with the maximum width and the maximum height.
	friend	tdim2		max( const tdim2& a, const tdim2& b ) { return tdim2(max(a.w,b.w),max(a.h,b.h)); }

	/// \brief Compute the minimum of two sizes.
	/// It returns the size consisted with the minimum width and the minimum height among two tdim2<T>.
	/// Note that the positive-constraint is not enforced.
	/// @param a The first tdim2<T>.
	/// @param b The second tdim2<T>.
	/// @return The tdim2<T> with the minimum width and the minimum height.
	friend	tdim2		min( const tdim2& a, const tdim2& b ) { return tdim2(min(a.w,b.w),min(a.h,b.h)); }
};

using dim2 = tdim2<float>;
using idim2 = tdim2<int>;

/// \brief Heterogeneous multiplication between tdim2 and scalar, which scales the size of tdim2
/// Multiplication of idim2, d and float, s produce dim2(d.w*s, d.h*s).
/// @param d The idim2 to scale.
/// @param s The scale facor.
/// @return The dim2 with the minimum width and the minimum height.
inline dim2 operator * (const idim2& d,float s) { return dim2(d.w*s,d.h*s); }

/// \brief Heterogeneous multiplication between tdim2 and scalar, which scales the size of tdim2
/// Multiplication of idim2, d and float, s produce dim2(d.w*s, d.h*s).
/// @param s The scale facor.
/// @param d The idim2 to scale.
/// @return The dim2 with the minimum width and the minimum height.
inline dim2 operator * (float s,const idim2& d) { return dim2(d.w*s,d.h*s); }

/// \brief Heterogeneous multiplication between tdim2 and scalar, which scales the size of tdim2
/// Multiplication of idim2, d and int, s produce dim2(d.w*s, d.h*s).
/// @param s The scale facor.
/// @param d The dim2 to scale.
/// @return The dim2 with the minimum width and the minimum height.
inline dim2 operator * (const dim2& d,int s) { return dim2(d.w*s,d.h*s); }

/// \brief Heterogeneous multiplication between tdim2 and scalar, which scales the size of tdim2
/// Multiplication of idim2, d and int, s produce dim2(d.w*s, d.h*s).
/// @param s The scale facor.
/// @param d The dim2 to scale.
/// @return The dim2<T> with the minimum width and the minimum height.
inline dim2 operator * (int s,const dim2& d) { return dim2(d.w*s,d.h*s); }


/// \brief 2D rectangle
///
/// A structure (template) to store 2D rectangle.
template<typename T> struct trect {
	union {
		struct {
			T x,		///< The x coordinate of the top-left corner.
			y,			///< The y coordinate of the top-left corner.
			w,			///< The width of the rectangle.
			h;			///< The height of the rectangle.
		};
		struct {
			vec<2,T> p;	///< The top-left corner of the rectangle.
			tdim2<T> sz;///< The size of the rectangle.
		};
		struct {
			T pt[4];	///< C style array representation.
						///< pt[0], pt[1], pt[2], and pt[3] corresponds to
						///< x, y, w, and h, respectively.
		};
	};
	/// \brief Copy constructor.
	///
	/// The simple copy constructor (+type converter) of trect<T>.
	template<typename T2>	trect( const trect<T2>& r )
	: x(r.x), y(r.y), w(r.w), h(r.h) {}

	
	/// \brief Default constructor.
	///
	/// The constructor of trect<T> with individual properties.
	/// @param a The x coordinate of the top-left corner.
	/// @param b The y coordinate of the top-left corner.
	/// @param c The width of the rectangle.
	/// @param d The height of the rectangle.
	inline				trect( const T& a=0, const T& b=0, const T& c=0, const T& d=0 )
	: x(a), y(b), w(c), h(d) {}

	/// \brief Constructor.
	///
	/// The constructor of trect<T> with the coordinate of the top-left corner and the size.
	/// @param a The x coordinate of the top-left corner.
	/// @param b The y coordinate of the top-left corner.
	/// @param ss The size of the rectangle.
	inline				trect( const T& a, const T& b, const tdim2<T>& ss )
	: x(a), y(b), w(ss.w), h(ss.h) {}

	/// \brief Constructor.
	///
	/// The constructor of trect<T> with the top-left corner and the size.
	/// @param pp The coordinate of the top-left corner.
	/// @param ss The size of the rectangle.
	inline				trect( const vec<2,T>& pp, const tdim2<T>& ss )
	: p(pp), sz(ss) {}

	/// \brief Constructor.
	///
	/// The constructor of trect<T> with the top-left corner, the width, and the height.
	/// @param pp The coordinate of the top-left corner.
	/// @param c The width of the rectangle.
	/// @param d The height of the rectangle.
	inline				trect( const vec<2,T>& pp, const T& c=0, const T& d=0 )
	: x(pp.x), y(pp.y), w(c), h(d) {}

	/// \brief Constructor.
	///
	/// The constructor of trect<T> that defined by two points.
	/// The resulting trect<T> is the minimum rectangle that includes two points.
	/// Note that resulting size always are not negative regardlessly to the relation between two points.
	/// @param a One point.
	/// @param b Another point.
	inline				trect( const vec<2,T>& a, const vec<2,T>& b )
	: x(min(a.x,b.x)), y(min(a.y,b.y)), w(abs(b.x-a.x)), h(abs(b.y-a.y)) {}

	/// Array style access operator.
	/// @param i the index of the array.
	inline T&			operator[] (int i) { return pt[i]; }

	/// Array style const access operator.
	/// @param i the index of the array.
	inline T operator[] (int i) const { return p[i]; }
	
	/// The top-left corner of the rectangle.
	inline vec<2,T>		tl() const { return p; }
	
	/// The top-right corner of the rectangle.
	inline vec<2,T>		tr() const { return vec<2,T>(x+w,y); }

	/// The bottom-left corner of the rectangle.
	inline vec<2,T>		bl() const { return vec<2,T>(x  ,y+h); }

	/// The bottom-right corner of the rectangle.
	inline vec<2,T>		br() const { return vec<2,T>(x+w,y+h); }

	/// The size of the rectangle.
	inline tdim2<T>		wh() const { return tdim2<T>(w,h); }

	/// The top-left corner of the rectangle.
	/// Note that this variation returns the assignable (reference) vec<2,T>.
	/// return The assignable top-left corner of the rectangle
	inline vec<2,T>&	tl() { return p; }
	
	/// The size of the rectangle.
	/// Note that this variation returns the assignable (reference) tdim2<T>.
	/// return The assignable size of the rectangle
	inline tdim2<T>&	wh() { return sz; }
	
	/// \brief Inclusion test.
	///
	/// Test if a point is in the rectangle.
	/// @param p The point to test.
	/// @return True if the point is in the rectangle.
	inline bool			in( const vec<2,T>& p ) const { return (p.x>=x)&&(p.y>=y)&&(p.x<x+w)&&(p.y<y+h); }

	/// \brief Increment function.
	///
	/// This function minimally increase the rectangle such that it includes the given point.
	/// @param p The point that should be included in the rectangle.
	/// @return Self reference.
	inline trect&		increase( const vec<2,T>& p ) {
		if( p.x<x ) { w+=x-p.x; x=p.x; }
		if( p.y<y ) { h+=y-p.y; y=p.y; }
		if( p.x>x+w ) { w=p.x-x; }
		if( p.y>y+h ) { h=p.y-y; }
		return *this;
	}

	/// \brief Increment function.
	///
	/// This function minimally increase the rectangle such that it includes the given rectangle.
	/// @param a The new rectangle that should be included in the rectangle.
	/// @return Self reference.
	inline trect&		increase( const trect<T>& a ) {
		T l = glm::min(x,a.x);
		T r = glm::max(x+w,a.x+a.w);
		T t = glm::min(y,a.y);
		T b = glm::max(y+h,a.y+a.h);
		x=l; y=t; w=r-l; h=b-t;
		return *this;
	}

//	inline trect		increased( const vec<2,T>& p ) const { trect ret = *this; ret.increase(p); return ret;}
//	inline trect		increased( const trect<T>& r ) const { trect ret = *this; ret.increase(r); return ret;}

	/// \brief Equality comparison operator
	///
	/// Test if the two rectangle are the same. (Both the sizes and the potisions are the same.)
	/// @param r The new rectangle to compare with.
	/// @return True if the rectangle and r are the same.
	inline bool			operator == (const trect<T>& r) const { return w==r.w && h==r.h && x==r.x && y==r.y; }

	/// \brief Inequality comparison operator
	///
	/// Test if the two rectangle are the same. (Both the sizes and the potisions are the same.)
	/// @param r The new rectangle to compare with.
	/// @return True if the rectangle and r are not the same.
	inline bool			operator != (const trect<T>& r) const { return w!=r.w || h!=r.h || x!=r.x || y!=r.y; }
};

template<typename T>
GLM_FUNC_QUALIFIER T const* value_ptr(trect<T> const& v) { return &(v.x); }
template<typename T>
GLM_FUNC_QUALIFIER T* value_ptr(trect<T>& v) { return &(v.x); }

using irect = trect<int>;
using rect = trect<float>;


} // End of Namespace glm


#endif /* JGL_Math_hpp */
