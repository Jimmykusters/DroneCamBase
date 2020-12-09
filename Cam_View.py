import math

def f(h, alpha, beta, input_is_degree=False):
    if input_is_degree:
        alpha = math.radians(alpha)
        beta = math.radians(beta)

    c = h/math.cos(alpha/2)
    l1 = 2*(math.sqrt(math.pow(c, 2) - math.pow(h, 2)))
    # l1 = (c*2 - h)*0.5 * 2
    print("l1 = " + str(l1))

    c2 = h/math.cos(beta/2)
    l2 = 2*(math.sqrt(math.pow(c2, 2) - math.pow(h, 2)))
    # l2 = (c2*2 - h)*0.5 * 2
    print("l2 = " + str(l2))
    return l1 * l2


print(f(90, 53.50, 41.41, True))