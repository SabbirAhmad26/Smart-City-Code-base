from scipy.optimize import minimize

class Control:
    def __init__(self, mode):
        self.mode = mode

    def control(self):
        def ellipse_safety_1(X_1):
            aH = 0.6
            bH = 0.1
            c1 = X_1[7]**2 - (X_1[4] - X_1[0] - X_1[8])**2 / (aH**2) - (X_1[5] - X_1[1] - X_1[9])**2 / (bH**2)
            return c1

        result = minimize(
            fun=lambda x: objective(x),
            x0=x_init,
            constraints={'type': 'ineq', 'fun': lambda x: ellipse_safety_1(x)},
            bounds=[(v_tk - s1, v_tk + s1), (x_tk - s2, x_tk + s2), (vl_tk - s1, vl_tk + s1), (xl_tk - s2, xl_tk + s2)]
        )

        fval_quad = result["fun"]


        def ellipse_safety_2(X2):
            aC = 0.6
            bC = 0.1
            c2 = X2[7]**2 - (X2[0]-X2[4])**2/aC**2 - (X2[1]-X2[5])^2/bC**2
            return c2

        def ellipse_safety_3(X3):
            aH = 0.6
            bH = 0.1
            c3 = X3[7] ** 2 - (X3[4] - X3[0] - X3[8]) ** 2 / (aH ** 2) - (X3[5] - X3[1] - X3[9]) ** 2 / (bH ** 2)
            return c3

        def ellipse_safety_U(X):
            aC = 0.6
            bC = 0.1
            cU = X[7] ** 2 - (X[0] - X[4]) ** 2 / (aC ** 2) - (X[1] - X[5]) ** 2 / (bC ** 2)
            return cU

