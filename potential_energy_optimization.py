import numpy as np

def energy(q1,q2,q3):
    x1 = np.array( [ [ 0 ],
                     [ 0 ] ] )
    x2 = np.array( [ [ q1 ],
                     [ q2 ] ] )
    x3 = np.array( [ [ 0 ],
                     [ d ] ] )
    x4 = np.array( [ [ q1 - d * np.cos( q3 ) ],
                     [ q2 + d * np.sin( q3 ) ] ] )

    P = k * (np.linalg.norm( x1 - x2 ) - l) ** 2 / 2 + k * (np.linalg.norm( x2 - x3 ) - l) ** 2 / 2 + \
        k * (np.linalg.norm( x3 - x4 ) - l) ** 2 / 2 + k * (np.linalg.norm( x4 - x1 ) - l) ** 2 / 2

q1 = 0.2
q2 = 0.2
q3 = 0.5
d = 1
l = 1
k = 10
alpha = 100
for i in range(100):
    j1 = (k*(l - ((q1 - d + d*np.sin(q3))**2 + (q1 - d*np.cos(q3))**2)**(1/2))*(2*d - 4*q1 + 2*d*np.cos(q3) - 2*d*np.sin(q3)))/(2*((q1 - d + d*np.sin(q3))**2 + (q1 - d*np.cos(q3))**2)**(1/2)) - (k*q1*(l - (q1**2 + q2**2)**(1/2)))/(q1**2 + q2**2)**(1/2) - (k*q1*(l - (q1**2 + (d - q2)**2)**(1/2)))/(q1**2 + (d - q2)**2)**(1/2) - (k*(l - ((q1 - d*np.cos(q3))**2 + (q1 + d*np.sin(q3))**2)**(1/2))*(4*q1 - 2*d*np.cos(q3) + 2*d*np.sin(q3)))/(2*((q1 - d*np.cos(q3))**2 + (q1 + d*np.sin(q3))**2)**(1/2))
    j2 = 2*k*q2 - d*k - (k*l*q2)/(q1**2 + q2**2)**(1/2) + (d*k*l)/(q1**2 + (d - q2)**2)**(1/2) - (k*l*q2)/(q1**2 + (d - q2)**2)**(1/2)
    j3 = - (d*k*(l - ((q1 - d + d*np.sin(q3))**2 + (q1 - d*np.cos(q3))**2)**(1/2))*(q1*np.cos(q3) - d*np.cos(q3) + q1*np.sin(q3)))/((q1 - d + d*np.sin(q3))**2 + (q1 - d*np.cos(q3))**2)**(1/2) - (2**(1/2)*d*k*q1*np.sin(q3 + np.pi/4)*(l - ((q1 - d*np.cos(q3))**2 + (q1 + d*np.sin(q3))**2)**(1/2)))/((q1 - d*np.cos(q3))**2 + (q1 + d*np.sin(q3))**2)**(1/2)

    q1 -= 1 / alpha * j1
    q2 -= 1 / alpha * j2
    q3 -= 1 / alpha * j3
    print(q1,q2,q3)