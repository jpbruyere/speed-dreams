#ifndef _SPAN_H_
#define _SPAN_H_

class Span  
{
public:
	Span();
	Span( const Span& span );
	Span( double A, double B );
	~Span();

	bool	IsNull() const;
	double	GetSize() const;

	void	Set( double x, double y );

	bool	Overlaps( const Span& span ) const;
	bool	Contains( const Span& span ) const;
	bool	Contains( double x ) const;

	Span	Intersect( const Span& span ) const;
	Span	Intersect( double A, double B ) const;

	void	Extend( double x );
	void	ExcludeLeftOf( double x );
	void	ExcludeRightOf( double x );

public:
	double	a;
	double	b;
};

#endif
