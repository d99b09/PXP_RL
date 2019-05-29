import math as m
import random as r

def coord_to_polar(x, y, z):
    r = m.sqrt(x**2 + y**2 + z**2)
    if x == 0 and y == 0 and z == 0:
        return 0, 0, 0
    alf = m.asin(z / r)
    c1 = m.cos(alf)

    if z == 0:
        if (x > 0) and (y > 0):
            fi = m.asin(y / r)
        elif (x > 0) and (y < 0):
            fi = -m.acos(x / r)
        elif (x < 0) and (y > 0):
            fi = m.acos(x / r)
        elif (x < 0) and (y < 0):
            fi = -m.acos(x / r)

        elif (x == 0) and (y != 0):
            if y > 0:
                fi = m.pi / 2
            elif y < 0:
                fi = -m.pi / 2
            else:
                fi = 0
        elif (y == 0) and (x != 0):
            if x > 0:
                fi = 0
            elif x < 0:
                fi = m.pi
    elif (x == 0) and (y == 0):
        fi = 0
    elif (x == 0) and (y != 0):
        if y > 0:
            fi = m.pi/2
        elif y < 0:
            fi = -m.pi/2
        else:
            fi = 0
    elif (y == 0) and (x != 0):
        if x > 0:
            fi = 0
        elif x < 0:
            fi = m.pi
    elif (x > 0) and (y > 0):
        fi = m.asin(y/(r * c1))
    elif (x > 0) and (y < 0):
        fi = -m.acos(x/(r * c1))
    elif (x < 0) and (y > 0):
        fi = m.acos(x/(r * c1))
    elif (x < 0) and (y < 0):
        fi = -m.acos(x/(r * c1))
    return r, alf, fi
