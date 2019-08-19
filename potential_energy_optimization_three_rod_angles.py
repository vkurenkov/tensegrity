import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(precision=5)
np.set_printoptions(suppress=True)

def potential_e(x1, x2, l):
    return k * (np.linalg.norm( x1 - x2 ) - l) ** 2 / 2


def energy(q1, q2, l, lambda1, lambda2):
    x11 = np.array( [ [ 0 ],
                      [ 0 ],
                      [ 0 ] ] )

    x12 = np.array( [ [ 0 ],
                      [ 0 ],
                      [ d ] ] )

    x21 = np.array( [ [ q1[ 0 ] ],
                      [ q1[ 1 ] ],
                      [ q1[ 2 ] ] ] )

    x22 = np.array( [ [ q1[ 0 ] + q1[ 3 ] ],
                      [ q1[ 1 ] + q1[ 4 ] ],
                      [ q1[ 2 ] + q1[ 5 ] ] ] )

    x31 = np.array( [ [ q2[ 0 ] ],
                      [ q2[ 1 ] ],
                      [ q2[ 2 ] ] ] )

    x32 = np.array( [ [ q2[ 0 ] + q2[ 3 ] ],
                      [ q2[ 1 ] + q2[ 4 ] ],
                      [ q2[ 2 ] + q2[ 5 ] ] ] )

    P = potential_e( x11, x21, l[ 0 ] ) + potential_e( x21, x31, l[ 1 ] ) + potential_e( x31, x11, l[ 2 ] ) + \
        potential_e( x12, x22, l[ 3 ] ) + potential_e( x22, x32, l[ 4 ] ) + potential_e( x32, x12, l[ 5 ] ) + \
        potential_e( x11, x32, l[ 6 ] ) + potential_e( x31, x22, l[ 7 ] ) + potential_e( x21, x12, l[ 8 ] )
    return P

def get_configuration(l):

    q11 = 0.0343
    q12 = 0.085
    q13 = 0.0822
    q14 = -0.012
    q15 = -0.191

    q21 = 0.107
    q22 = -0.012
    q23 = 0.060
    q24 = -0.189
    q25 = -0.030

    l1 = l[0]
    l2 = l[1]
    l3 = l[2]
    l4 = l[3]
    l5 = l[4]
    l6 = l[5]
    l7 = l[6]
    l8 = l[7]
    l9 = l[8]

    alpha = 500

    for i in range( 4000 ):
        alpha = 500 + i
        j11 = 6 * k * q11 - 3 * k * q21 + 3 * d * k * np.sin( q15 ) - d * k * np.sin( q25 ) - (
                    k * l8 * abs( q11 - q21 + d * np.sin( q15 ) ) * np.sign( q11 - q21 + d * np.sin( q15 ) )) / (
                    abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) - (
                    k * l2 * abs( q11 - q21 ) * np.sign( q11 - q21 )) / (
                    abs( q11 - q21 ) ** 2 + abs( q12 - q22 ) ** 2 + abs( q13 - q23 ) ** 2) ** (1 / 2) - (
                    k * l1 * abs( q11 ) * np.sign( q11 )) / (abs( q11 ) ** 2 + abs( q12 ) ** 2 + abs( q13 ) ** 2) ** (1 / 2) - (
                    k * l9 * abs( q11 ) * np.sign( q11 )) / (abs( q11 ) ** 2 + abs( q12 ) ** 2 + abs( d - q13 ) ** 2) ** (
                    1 / 2) - (k * l4 * abs( q11 + d * np.sin( q15 ) ) * np.sign( q11 + d * np.sin( q15 ) )) / (
                    abs( q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs(
                q11 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) - (k * l5 * abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) * np.sign(
            q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) )) / (abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
            q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2)

        j12 = 6 * k * q12 - 3 * k * q22 + 3 * d * k * np.cos( q15 ) * np.sin( q14 ) - d * k * np.cos( q25 ) * np.sin( q24 ) - (
                    k * l2 * abs( q12 - q22 ) * np.sign( q12 - q22 )) / (
                    abs( q11 - q21 ) ** 2 + abs( q12 - q22 ) ** 2 + abs( q13 - q23 ) ** 2) ** (1 / 2) - (
                    k * l1 * abs( q12 ) * np.sign( q12 )) / (abs( q11 ) ** 2 + abs( q12 ) ** 2 + abs( q13 ) ** 2) ** (1 / 2) - (
                    k * l5 * np.sign( q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) * abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) )) / (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2) - (
                    k * l9 * abs( q12 ) * np.sign( q12 )) / (abs( q11 ) ** 2 + abs( q12 ) ** 2 + abs( d - q13 ) ** 2) ** (
                    1 / 2) - (
                    k * l4 * abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) * np.sign( q12 + d * np.cos( q15 ) * np.sin( q14 ) )) / (
                    abs( q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs(
                q11 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) - (k * l8 * abs( q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) * np.sign(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) )) / (abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2)

        j13 = 6 * k * q13 - 2 * d * k - 3 * k * q23 + 3 * d * k * np.cos( q14 ) * np.cos( q15 ) - d * k * np.cos( q24 ) * np.cos( q25 ) - (
                    k * l5 * abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) * np.sign(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) )) / (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2) - (
                    k * l2 * abs( q13 - q23 ) * np.sign( q13 - q23 )) / (
                    abs( q11 - q21 ) ** 2 + abs( q12 - q22 ) ** 2 + abs( q13 - q23 ) ** 2) ** (1 / 2) - (
                    k * l1 * abs( q13 ) * np.sign( q13 )) / (abs( q11 ) ** 2 + abs( q12 ) ** 2 + abs( q13 ) ** 2) ** (1 / 2) - (
                    k * l8 * abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) * np.sign(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) )) / (abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) + (
                    k * l9 * np.sign( d - q13 ) * abs( d - q13 )) / (abs( q11 ) ** 2 + abs( q12 ) ** 2 + abs( d - q13 ) ** 2) ** (
                    1 / 2) - (k * l4 * abs( q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) * np.sign(
            q13 - d + d * np.cos( q14 ) * np.cos( q15 ) )) / (
                    abs( q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs(
                q11 + d * np.sin( q15 ) ) ** 2) ** (1 / 2)

        j14 = - (d * k * np.cos( q15 ) * (l4 - (abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 + d * np.sin( q15 ) ) ** 2 + abs(
            q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2) ** (1 / 2)) * (
                       q12 * np.cos( q14 ) + d * np.sin( q14 ) - q13 * np.sin( q14 ))) / (
                    abs( q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs(
                q11 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) - (d * k * np.cos( q15 ) * (l8 - (
                    abs( q11 - q21 + d * np.sin( q15 ) ) ** 2 + abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2) ** (1 / 2)) * (
                                                                     q12 * np.cos( q14 ) - q22 * np.cos( q14 ) - q13 * np.sin(
                                                                 q14 ) + q23 * np.sin( q14 ))) / (
                    abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) - (
                    d * k * np.cos( q15 ) * (l5 - (
                        abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                    q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                    q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2) ** (1 / 2)) * (
                                q12 * np.cos( q14 ) - q22 * np.cos( q14 ) - q13 * np.sin( q14 ) + q23 * np.sin( q14 ) - d * np.cos(
                            q14 ) * np.cos( q25 ) * np.sin( q24 ) + d * np.cos( q24 ) * np.cos( q25 ) * np.sin( q14 ))) / (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2)

        j15 = (k * (l5 - (abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
            q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2) ** (1 / 2)) * (2 * d * np.sin( q14 ) * np.sin( q15 ) * (
                    q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 )) - 2 * d * np.cos( q15 ) * (
                                                                                         q11 - q21 + d * np.sin(
                                                                                     q15 ) - d * np.sin( q25 )) + 2 * d * np.cos(
            q14 ) * np.sin( q15 ) * (q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 )))) / (2 * (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2)) + (k * (l4 - (
                    abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 + d * np.sin( q15 ) ) ** 2 + abs(
                q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2) ** (1 / 2)) * (2 * d * np.cos( q14 ) * np.sin( q15 ) * (
                    q13 - d + d * np.cos( q14 ) * np.cos( q15 )) - 2 * d * np.cos( q15 ) * (q11 + d * np.sin( q15 )) + 2 * d * np.sin(
            q14 ) * np.sin( q15 ) * (q12 + d * np.cos( q15 ) * np.sin( q14 )))) / (2 * (
                    abs( q13 - d + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs( q12 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs(
                q11 + d * np.sin( q15 ) ) ** 2) ** (1 / 2)) + (k * (l8 - (
                    abs( q11 - q21 + d * np.sin( q15 ) ) ** 2 + abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2) ** (1 / 2)) * (2 * d * np.cos( q14 ) * np.sin( q15 ) * (
                    q13 - q23 + d * np.cos( q14 ) * np.cos( q15 )) - 2 * d * np.cos( q15 ) * (q11 - q21 + d * np.sin(
            q15 )) + 2 * d * np.sin( q14 ) * np.sin( q15 ) * (q12 - q22 + d * np.cos( q15 ) * np.sin( q14 )))) / (2 * (
                    abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2))

        j21 = 6 * k * q21 - 3 * k * q11 - 2 * d * k * np.sin( q15 ) + 3 * d * k * np.sin( q25 ) + (
                    k * l8 * abs( q11 - q21 + d * np.sin( q15 ) ) * np.sign( q11 - q21 + d * np.sin( q15 ) )) / (
                    abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) + (
                    k * l2 * abs( q11 - q21 ) * np.sign( q11 - q21 )) / (
                    abs( q11 - q21 ) ** 2 + abs( q12 - q22 ) ** 2 + abs( q13 - q23 ) ** 2) ** (1 / 2) - (
                    k * l3 * abs( q21 ) * np.sign( q21 )) / (abs( q21 ) ** 2 + abs( q22 ) ** 2 + abs( q23 ) ** 2) ** (1 / 2) - (
                    k * l6 * abs( q21 + d * np.sin( q25 ) ) * np.sign( q21 + d * np.sin( q25 ) )) / (
                    abs( q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2) + (k * l5 * abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) * np.sign(
            q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) )) / (abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
            q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2) - (
                    k * l7 * abs( q21 + d * np.sin( q25 ) ) * np.sign( q21 + d * np.sin( q25 ) )) / (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2)

        j22 = 6 * k * q22 - 3 * k * q12 - 2 * d * k * np.cos( q15 ) * np.sin( q14 ) + 3 * d * k * np.cos( q25 ) * np.sin( q24 ) + (
                    k * l2 * abs( q12 - q22 ) * np.sign( q12 - q22 )) / (
                    abs( q11 - q21 ) ** 2 + abs( q12 - q22 ) ** 2 + abs( q13 - q23 ) ** 2) ** (1 / 2) - (
                    k * l3 * abs( q22 ) * np.sign( q22 )) / (abs( q21 ) ** 2 + abs( q22 ) ** 2 + abs( q23 ) ** 2) ** (1 / 2) + (
                    k * l5 * np.sign( q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) * abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) )) / (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2) - (
                    k * l6 * abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) * np.sign( q22 + d * np.cos( q25 ) * np.sin( q24 ) )) / (
                    abs( q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2) - (
                    k * l7 * abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) * np.sign( q22 + d * np.cos( q25 ) * np.sin( q24 ) )) / (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2) + (k * l8 * abs( q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) * np.sign(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) )) / (abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2)

        j23 = 6 * k * q23 - 3 * k * q13 - d * k - 2 * d * k * np.cos( q14 ) * np.cos( q15 ) + 3 * d * k * np.cos( q24 ) * np.cos( q25 ) + (
                    k * l5 * abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) * np.sign(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) )) / (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2) + (
                    k * l2 * abs( q13 - q23 ) * np.sign( q13 - q23 )) / (
                    abs( q11 - q21 ) ** 2 + abs( q12 - q22 ) ** 2 + abs( q13 - q23 ) ** 2) ** (1 / 2) - (
                    k * l3 * abs( q23 ) * np.sign( q23 )) / (abs( q21 ) ** 2 + abs( q22 ) ** 2 + abs( q23 ) ** 2) ** (1 / 2) - (
                    k * l7 * abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) * np.sign( q23 + d * np.cos( q24 ) * np.cos( q25 ) )) / (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2) + (k * l8 * abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) * np.sign(
            q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) )) / (abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) ) ** 2 + abs( q11 - q21 + d * np.sin( q15 ) ) ** 2) ** (1 / 2) - (
                    k * l6 * abs( q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) * np.sign(
                q23 - d + d * np.cos( q24 ) * np.cos( q25 ) )) / (
                    abs( q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2)

        j24 = (d * k * np.cos( q25 ) * (l5 - (abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
            q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
            q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2) ** (1 / 2)) * (
                     q12 * np.cos( q24 ) - q22 * np.cos( q24 ) - q13 * np.sin( q24 ) + q23 * np.sin( q24 ) - d * np.cos( q14 ) * np.cos(
                 q15 ) * np.sin( q24 ) + d * np.cos( q15 ) * np.cos( q24 ) * np.sin( q14 ))) / (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2) - (
                    d * k * np.cos( q25 ) * (l6 - (
                        abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs( q21 + d * np.sin( q25 ) ) ** 2 + abs(
                    q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2) ** (1 / 2)) * (
                                q22 * np.cos( q24 ) + d * np.sin( q24 ) - q23 * np.sin( q24 ))) / (
                    abs( q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2) - (d * k * np.cos( q25 ) * (l7 - (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2)) * (q22 * np.cos( q24 ) - q23 * np.sin( q24 ))) / (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2)

        j25 = (k * (l6 - (abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs( q21 + d * np.sin( q25 ) ) ** 2 + abs(
            q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2) ** (1 / 2)) * (
                     2 * d * np.cos( q24 ) * np.sin( q25 ) * (q23 - d + d * np.cos( q24 ) * np.cos( q25 )) - 2 * d * np.cos( q25 ) * (
                         q21 + d * np.sin( q25 )) + 2 * d * np.sin( q24 ) * np.sin( q25 ) * (q22 + d * np.cos( q25 ) * np.sin( q24 )))) / (
                    2 * (abs( q23 - d + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs( q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2)) - (k * (l5 - (
                    abs( q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2) ** (1 / 2)) * (2 * d * np.sin( q24 ) * np.sin( q25 ) * (
                    q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 )) - 2 * d * np.cos( q25 ) * (
                                                                                             q11 - q21 + d * np.sin(
                                                                                         q15 ) - d * np.sin(
                                                                                         q25 )) + 2 * d * np.cos( q24 ) * np.sin(
            q25 ) * (q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 )))) / (2 * (
                    abs( q11 - q21 + d * np.sin( q15 ) - d * np.sin( q25 ) ) ** 2 + abs(
                q13 - q23 + d * np.cos( q14 ) * np.cos( q15 ) - d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs(
                q12 - q22 + d * np.cos( q15 ) * np.sin( q14 ) - d * np.cos( q25 ) * np.sin( q24 ) ) ** 2) ** (1 / 2)) + (k * (l7 - (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2)) * (2 * d * np.cos( q24 ) * np.sin( q25 ) * (
                    q23 + d * np.cos( q24 ) * np.cos( q25 )) - 2 * d * np.cos( q25 ) * (q21 + d * np.sin( q25 )) + 2 * d * np.sin(
            q24 ) * np.sin( q25 ) * (q22 + d * np.cos( q25 ) * np.sin( q24 )))) / (2 * (
                    abs( q23 + d * np.cos( q24 ) * np.cos( q25 ) ) ** 2 + abs( q22 + d * np.cos( q25 ) * np.sin( q24 ) ) ** 2 + abs(
                q21 + d * np.sin( q25 ) ) ** 2) ** (1 / 2))

        q11 -= 1 / alpha * j11
        q12 -= 1 / alpha * j12
        q13 -= 1 / alpha * j13
        q14 -= 1 / alpha * j14
        q15 -= 1 / alpha * j15

        q21 -= 1 / alpha * j21
        q22 -= 1 / alpha * j22
        q23 -= 1 / alpha * j23
        q24 -= 1 / alpha * j24
        q25 -= 1 / alpha * j25

        #print(q14,q15,q24,q25)
    return np.array([[q11,q12,q13,q14,q15,q21,q22,q23,q24,q25]])

d = 0.2
k = 100
l = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
q0 = get_configuration(l)[0][0:5]
q = []
delta = 0.001
for i in range(len(l)):
    l[i]+=delta
    q.append(get_configuration( l )[0][0:5] - q0)
    l[i]-=delta
    print(i)
print(np.array(q).transpose()/delta)

'''
d = 0.1
K = [[  1.59710577   1.22687673   0.90745261   1.03421257   0.65389225 1.13247175   1.0120698    0.99374756   0.94931962]
     [  0.02303697  -0.12578371  -0.3124041   -0.14152612  -0.6403574 0.33045871  -0.44567909  -0.26604875  -0.42717396]
     [  0.60031265   0.22541587   0.08028436   0.09062223   0.1781778 0.02253926   0.21976847   0.10514886  -1.08245851]
     [  2.09710837  -7.03636795   7.85293575  11.09275627  10.0873367 -7.45474334   5.43615789   5.8301208    6.69036557]
     [-21.79196065 -18.38865361 -11.07300811 -21.64595946  -9.36525335 -11.63165009 -17.17976065 -15.89965169  -8.43504477]]

 d = 0.01
 K = [[  1.83579   0.90977  -0.79208  -1.06343  -0.59206   0.82667   0.36888 0.04059  -0.04941]
     [  1.98076   0.86751  -1.14716  -1.45897  -0.96057   0.83832  -0.03139 -0.2258    0.39279]
     [  0.25854   0.14424   0.19135   0.2221    0.17031   0.01276   0.12007 0.08619  -0.69546]
     [-30.00303 -26.96451  29.03166  30.89565  24.24952 -20.20718  -0.79319 3.96876   1.41576]
     [-16.20146  -8.19963   3.30861  11.08701   2.07346  -6.96281  -3.90078 -0.61254  -1.66077]]
     

 '''