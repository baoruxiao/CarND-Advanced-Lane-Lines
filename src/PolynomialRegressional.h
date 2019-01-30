/*
 * PolynomialRegressional.h
 *
 *  Created on: Jan 27, 2019
 *      Author: Bao
 */

#ifndef POLYNOMIALREGRESSIONAL_H_
#define POLYNOMIALREGRESSIONAL_H_

#include<vector>
#include<stdlib.h>


template <class TYPE>
class PolynomialRegression {
public:

	PolynomialRegression();
	virtual ~PolynomialRegression() {};

	bool fitIt(
			const std::vector<TYPE> & x,
			const std::vector<TYPE> & y,
			const int & order,
			std::vector<TYPE> &	coeffs);
};

template <class TYPE>
PolynomialRegression<TYPE>::PolynomialRegression() {};

template <class TYPE>
bool PolynomialRegression<TYPE>::fitIt(
		const std::vector<TYPE> & x,
		const std::vector<TYPE> & y,
		const int & order,
		std::vector<TYPE> & coeffs) {
	if (x.size() != y.size()) {
		throw std::runtime_error(
				"The size of x & y arrays must be the same" );
		return false;
	}
	if (x.size() == 0 || y.size() == 0) {
		throw std::runtime_error(
				"The size of x & y shouldn't be 0" );
		return false;
	}

	size_t N = x.size();
	int n = order;
	int np1 = n + 1;
	int np2 = n + 2;
	int tnp1 = 2 * n + 1;
	TYPE tmp;

	// X = vector that stores values of sigma(xi^2n)
	std::vector<TYPE> X(tnp1);
	for (int i = 0; i < tnp1; ++i) {
		X[i] = 0;
		for (int j = 0; j < N; ++j) {
			X[i] += (TYPE)pow(x[j], i);
		}
	}

	// a = vector to store final coefficients;
	std::vector<TYPE> a(np1);

	// B = normal augmented matrix that stores the equations.
	std::vector<std::vector<TYPE> > B(np1, std::vector<TYPE> (np2, 0));

	for (int i = 0; i <= n; ++i) {
		for (int j = 0; j <= n; ++j) {
			B[i][j] = X[i + j];
		}
	}

	// Y = vector to store values of sigma(xi^n * yi)
	std::vector<TYPE> Y(np1);
	for (int i = 0; i < np1; ++i) {
		Y[i] = (TYPE)0;
		for (int j = 0; j < N; ++j) {
			Y[i] += (TYPE)pow(x[j], i)*y[j];
		}
	}

	// Load values of Y as last column of B
	for (int i = 0; i <= n; ++i)
		B[i][np1] = Y[i];

	n += 1;
	int nm1 = n - 1;

	// Pivotisation of the matrix.
	for (int i = 0; i < n; ++i) {
		for (int k = i+1; k < n; ++k) {
			if (B[i][i] < B[k][i]) {
				for (int j = 0; j <= n; ++j) {
					tmp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = tmp;
				}
			}
		}
	}

	// Performs the Gaussian elimination
	// (1) Make all elements below the pivot equals to zero
	//   or eliminate the variable.
	for (int i = 0; i < nm1; ++i) {
		for (int k = i+1; k < n; ++k) {
			TYPE t = B[k][i] / B[i][i];
			for (int j = 0; j <= n; ++j) {
				B[k][j] -= t*B[i][j];	// (1)
			}
		}
	}

	// Back substitution.
	// (1) Set the variable as the rhs of the last equation
	// (2) Subtract all lhs values except the target coefficient.
	// (3) Divide rhs by coefficient of variable being calculated.
	for (int i = nm1; i >= 0; --i) {
		a[i] = B[i][n];
		for (int j = 0; j < n; ++j) {
			if (j != i) {
				a[i] -= B[i][j] * a[j];
			}
		}
		a[i] /= B[i][i];
	}

	coeffs.resize(a.size());
	for (size_t i = 0; i < a.size(); ++i) {
		coeffs[i] = a[i];
	}

	return true;
}


#endif /* POLYNOMIALREGRESSIONAL_H_ */
