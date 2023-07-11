import numpy as np
from scipy.special import comb
from sympy import symbols, Poly
import matplotlib.pyplot as plt


def bernstein_polynomials_coeffs(p):
    dimPolynomial = p.shape[1]
    t = symbols('t')

    B = np.zeros((dimPolynomial, dimPolynomial), dtype=object)

    for i in range(1, dimPolynomial + 1):
        coeff = comb(dimPolynomial - 1, i - 1)
        polynomial = coeff * t ** (i - 1) * (1 - t) ** (dimPolynomial - 1 - (i - 1))
        Expression = Poly(polynomial, t)
        poly_array = np.array(Expression.all_coeffs())
        B[i - 1, :] = poly_array

    a = np.dot(p, B)
    return a


def plot_polynomials(p):
    x = np.linspace(-10, 10, 100)  # Adjust the range and number of points as needed

    # Evaluate the polynomial for each x value
    y = np.polyval(p, x)

    # Plot the polynomial curve
    plt.plot(x, y)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Polynomial Plot')
    plt.grid(True)
    plt.show()


p = np.array([[288, 288, 287, 328, 447], [447, 329, 287, 287, 288]])
coefficients = bernstein_polynomials_coeffs(p)
print(coefficients)

plot_polynomials(coefficients[0])
