#pragma once

#include <cmath>


namespace maytag::_
{
	// Computes the cholesky factorization of a, putting the lower triangular matrix into r.
	// a - upper triangular matrix.
	void mat33_chol(const double* const a, double* const r)
	{
		// a[0] = r[0]*r[0]
		r[0] = std::sqrt(a[0]);
		// a[1] = r[0]*r[3]
		r[3] = a[1] / r[0];
		// a[2] = r[0]*r[6]
		r[6] = a[2] / r[0];
		// a[4] = r[3]*r[3] + r[4]*r[4]
		r[4] = std::sqrt(a[3] - r[3] * r[3]);
		// a[5] = r[3]*r[6] + r[4]*r[7]
		r[7] = (a[4] - r[3] * r[6]) / r[4];
		// a[8] = r[6]*r[6] + r[7]*r[7] + r[8]*r[8]
		r[8] = std::sqrt(a[5] - r[6] * r[6] - r[7] * r[7]);
		r[1] = 0.0;
		r[2] = 0.0;
		r[5] = 0.0;
	}

	void mat33_lower_tri_inv(const double* const a, double* const r)
	{
		// a[0]*r[0] = 1
		r[0] = 1.0 / a[0];
		// a[3]*r[0] + a[4]*r[3] = 0
		r[3] = -a[3] * r[0] / a[4];
		// a[4]*r[4] = 1
		r[4] = 1.0 / a[4];
		// a[6]*r[0] + a[7]*r[3] + a[8]*r[6] = 0
		r[6] = (-a[6] * r[0] - a[7] * r[3]) / a[8];
		// a[7]*r[4] + a[8]*r[7] = 0
		r[7] = -a[7] * r[4] / a[8];
		// a[8]*r[8] = 1
		r[8] = 1 / a[8];
	}

	// a - upper triangular matrix.
	void mat33_sym_solve(const double* const a, const double* const b, double* const r)
	{
		double t[9];
		mat33_chol(a, t);
		double m[9];
		mat33_lower_tri_inv(t, m);
		//
		t[0] = m[0] * b[0];
		t[1] = m[3] * b[0] + m[4] * b[1];
		t[2] = m[6] * b[0] + m[7] * b[1] + m[8] * b[2];
		//
		r[0] = m[0] * t[0] + m[3] * t[1] + m[6] * t[2];
		r[1] = m[4] * t[1] + m[7] * t[2];
		r[2] = m[8] * t[2];
	}

	// Regresses a model of the form:
	// intensity(x,y) = c0*x + c1*y + c2
	// The J matrix is the:
	//     |x1 y1 1|
	// J = |x2 y2 1|
	//     |  ...  |
	// The a matrix is J'J
	struct graymodel_t
	{
		// a - upper triangular matrix.
		// | 0 1 2 |
		// |   3 4 |
		// |     5 |
		double a[6];
		double b[3];
		double c[3];

		graymodel_t()
		{
			std::memset(this, 0, sizeof(graymodel_t));
		}

		void add(double x, double y, double gray)
		{
			// Update upper right entries of A = J'J.
			a[0] += x * x;
			a[1] += x * y;
			a[2] += x;
			a[3] += y * y;
			a[4] += y;
			a[5] += 1.0;
			// Update B = J'gray.
			b[0] += x * gray;
			b[1] += y * gray;
			b[2] += gray;
		}

		void solve()
		{
			mat33_sym_solve(a, b, c);
		}

		double interpolate(double x, double y)
		{
			return c[0] * x + c[1] * y + c[2];
		}
	};
}