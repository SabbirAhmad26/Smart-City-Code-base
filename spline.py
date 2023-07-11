import math
import numpy as np
from scipy.special import comb
from sympy import symbols, Poly


def poly_calc(pts):
    p = []
    p1 = []
    p2 = []
    for i in pts:
        p1.append(i[0])
        p2.append(i[1])
    p.append(p1)
    p.append(p2)
    
    dimPolynomial = len(p[0])
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

# p = poly_calc([(287, 431), (287, 328), (290, 312), (299, 298), (312, 288), (328, 287), (441, 281)])
# print(p[0])