#ifndef _QUADRATIC_H_
#define _QUADRATIC_H_

class Quadratic  
{
public:
	Quadratic();
	Quadratic( double a, double b, double c );
	Quadratic( double x, double y, double velY, double accY );
	~Quadratic();

	void		Setup( double a, double b, double c );
	void		Setup( double x, double y, double velY, double accY );

	double		CalcMin() const;
	double		CalcY( double x ) const;
	bool		Solve( double y, double& x0, double& x1 ) const;
	bool		SmallestNonNegativeRoot( double& t ) const;

	Quadratic	operator+( const Quadratic& q ) const;
	Quadratic	operator-( const Quadratic& q ) const;

private:
	double		m_a;
	double		m_b;
	double		m_c;
};

#endif
