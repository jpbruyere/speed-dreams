/***************************************************************************

    file                 : straight2_t.h
    created              : Due Apr 5 13:51:00 CET 2005
    copyright            : (C) 2005 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id$

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

/*
	Template for 2d-straight, to be used with float or double 2-d vectors. This template is NOT
	intended to work with classes which allocate memory. Be aware that there are more
	efficient methods for doing most of the operations (avoiding temp verctors and make
	better use of registers). Later I will try to improve the performance (SSE,
	"abuse" of templates to avoid temporaries).
*/


#ifndef _STRAIGHT_2T_H_
#define _STRAIGHT_2T_H_

#include <tgf.h>

template<class T> class straight2t {
    public:
        // Constructors.
        straight2t() {}
        straight2t(T x, T y, T dx, T dy)
            { p.x = x; p.y = y; d.x = dx; d.y = dy; glm::normalize(d); }
        straight2t(const glm::tvec2<T> &anchor, const glm::tvec2<T> &dir)
            { p = anchor; d = dir; glm::normalize(d); }

        // Methods.
        glm::tvec2<T> intersect(const straight2t<T> &s) const;		// Intersection of 2 straights: does not check for NaN's!
        T dist(const glm::tvec2<T> &p) const;						// Distance of p to straight this.

        // Data.
        glm::tvec2<T> p;	// Point on the straight.
        glm::tvec2<T> d;	// Direction of the straight.
};


// intersection point of *this and s
template<class T> inline glm::tvec2<T> straight2t<T>::intersect(const straight2t<T> &s) const
{
    T t = -(d.x*(s.p.y-p.y)+d.y*(p.x-s.p.x))/(d.x*s.d.y-d.y*s.d.x);
    return s.p + s.d*t;
}


// distance of point s from straight *this
template<class T> inline T straight2t<T>::dist(const glm::tvec2<T> &s) const
{
    glm::tvec2<T> d1 = s - p;
    glm::tvec2<T> d3 = d1 - d*d1*d;
    return d3.length();
}

#endif //_STRAIGHT_2T_H_


