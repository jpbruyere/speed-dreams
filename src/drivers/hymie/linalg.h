/***************************************************************************

    file                 : linalg.h
    created              : Mo Mar 11 13:51:00 CET 2002
    copyright            : (C) 2002 by Bernhard Wymann
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
 
#ifndef _LINALG_H_
#define _LINALG_H_

inline float sign(float d) {
 return (d >= 0.0) ? 1.0f : -1.0f;
}


class v2d {
 public:
  /* constructors */
  v2d() {}
  v2d(const v2d &src) { this->x = src.x; this->y = src.y; }
  v2d(float x, float y) { this->x = x; this->y = y; }

  /* operators */
  v2d& operator=(const v2d &src);         /* assignment */
  v2d operator+(const v2d &src) const;    /* addition */
  v2d operator-(void) const;              /* negation */
  v2d operator-(const v2d &src) const;    /* subtraction */
  v2d operator*(const float s) const;     /* multiply with scalar */
  float operator*(const v2d &src) const;  /* dot product */
  friend v2d operator*(const float s, const v2d & src);

  /* methods */
  float len(void) const;
  void normalize(void);
  float dist(const v2d &p) const;
  float cosalpha(const v2d &p2, const v2d &center) const;
  v2d rotate(const v2d &c, float arc) const;

  /* data */
  float x;
  float y;
};


/* assignment */
inline v2d& v2d::operator=(const v2d &src)
{
    x = src.x; y = src.y; return *this;
}


/* add *this + src (vector addition) */
inline v2d v2d::operator+(const v2d &src) const
{
    return v2d(x + src.x, y + src.y);
}


/* negation of *this */
inline v2d v2d::operator-(void) const
{
    return v2d(-x, -y);
}


/* compute *this - src (vector subtraction) */
inline v2d v2d::operator-(const v2d &src) const
{
    return v2d(x - src.x, y - src.y);
}


/* scalar product */
inline float v2d::operator*(const v2d &src) const
{
    return src.x*x + src.y*y;
}


/* multiply vector with scalar (v2d*float) */
inline v2d v2d::operator*(const float s) const
{
    return v2d(s*x, s*y);
}


/* multiply scalar with vector (float*v2d) */
inline v2d operator*(const float s, const v2d & src)
{
    return v2d(s*src.x, s*src.y);
}


/* compute cosine of the angle between vectors *this-c and p2-c */
inline float v2d::cosalpha(const v2d &p2, const v2d &c) const
{
 v2d l1 = *this-c;
 v2d l2 = p2 - c;
 return (l1*l2)/(l1.len()*l2.len());
}


/* rotate vector arc radians around center c */
inline v2d v2d::rotate(const v2d &c, float arc) const
{
 v2d d = *this-c;
 float sina = sin(arc), cosa = cos(arc);
 return c + v2d(d.x*cosa-d.y*sina, d.x*sina+d.y*cosa);
}


/* compute the length of the vector */
inline float v2d::len(void) const
{
 return sqrt(x*x+y*y);
}


/* distance between *this and p */
inline float v2d::dist(const v2d &p) const
{ 
 return sqrt((p.x-x)*(p.x-x)+(p.y-y)*(p.y-y));
}


/* normalize the vector */
inline void v2d::normalize(void)
{
 float l = this->len();
 x /= l; y /= l;
}


class Straight {
    public:
        /* constructors */
        Straight() {}
        Straight(float x, float y, float dx, float dy)
            {  p.x = x; p.y = y; d.x = dx; d.y = dy; d.normalize(); }
        Straight(const v2d &anchor, const v2d &dir)
            { p = anchor; d = dir; d.normalize(); }

        /* methods */
        v2d intersect(const Straight &s) const;
        float dist(const v2d &p) const; 

        /* data */
        v2d p;          /* point on the straight */
        v2d d;          /* direction of the straight */
};


/* intersection point of *this and s */
inline v2d Straight::intersect(const Straight &s) const
{
    float t = -(d.x*(s.p.y-p.y)+d.y*(p.x-s.p.x))/(d.x*s.d.y-d.y*s.d.x);
    return s.p + s.d*t;  
}


/* distance of point s from straight *this */
inline float Straight::dist(const v2d &s) const
{
    v2d d1 = s - p;
    v2d d3 = d1 - d*d1*d;
    return d3.len();
}

class v3d {
 public:
  double x, y, z;  /* coordinates */

  /* constructors */
  v3d() {}
  v3d(const v3d &src) { this->x = src.x; this->y = src.y; this->z = src.z; }
  v3d(double x, double y, double z) { this->x = x; this->y = y; this->z = z; }

  /* operators */
  v3d & operator=(const v3d &src);
  v3d operator-(void);
  v3d operator-(const v3d &src);
  v3d operator+(const v3d &src);
  v3d operator*(const double s);       /* multiply by scalar */
  v3d operator/(const double s);       /* divide by scalar */
  double operator*(const v3d &src);      /* dot product */
  friend v3d operator*(const double s, const v3d & src); /* multiply by scalar */

  /* other methods */
  double len(void);
  void normalize(void);
  void crossProduct(const v3d* b, v3d* r);    /* r := this X b */
  void dirVector(const v3d* b, v3d* r);     /* r := this - b */
};


inline v3d & v3d::operator=(const v3d &src)
{
 x = src.x; y = src.y; z = src.z; return *this;
}


inline v3d v3d::operator-(void)
{
 return v3d(-this->x, -this->y, -this->z);
}


inline v3d v3d::operator-(const v3d &src)
{
 return v3d(this->x - src.x, this->y - src.y, this->z - src.z);
}


inline v3d v3d::operator+(const v3d &src)
{
 return v3d(this->x + src.x, this->y + src.y, this->z + src.z);
}


inline v3d v3d::operator*(const double s)
{
 return v3d(s*this->x, s*this->y, s*this->z);
}


inline double v3d::operator*(const v3d &src)
{
 return this->x*src.x + this->y*src.y + this->z*src.z;
}


inline v3d operator*(const double s, const v3d & src)
{
 return v3d(s*src.x, s*src.y, s*src.z);
}


inline v3d v3d::operator/(const double s)
{
 return v3d(this->x/s, this->y/s, this->z/s);
}


inline double v3d::len(void)
{
 return sqrt(x*x + y*y + z*z);
}


inline void v3d::normalize(void)
{
 double len = this->len();
 x /= len; y /= len; z /= len;
}


inline void v3d::crossProduct(const v3d* b, v3d* r) {
 r->x = this->y*b->z - this->z*b->y;
 r->y = this->z*b->x - this->x*b->z;
 r->z = this->x*b->y - this->y*b->x;
}

inline void v3d::dirVector(const v3d* b, v3d* r) {
 r->x = this->x - b->x; r->y = this->y - b->y; r->z = this->z - b->z;
}

#endif // _LINALG_H_

