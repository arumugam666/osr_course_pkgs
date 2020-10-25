import numpy as np
D1 = 0.1
D2 = 0.15

def fk(theta1, theta2):
    x = D1*np.cos(theta1)+ D2*np.cos(theta1+theta2)
    y = D1*np.sin(theta1)+ D2*np.sin(theta1+theta2)
    theta = theta1+theta2
    return (x,y,theta)

def jacobian(theta1, theta2):
    matrix = np.ones((3,2))
    matrix[0,0] = - D1*np.sin(theta1) - D2*np.sin(theta1+theta2)
    matrix[0,1] = - D2*np.sin(theta1+theta2)
    matrix[1,0] = D1*np.cos(theta1)+ D2*np.cos(theta1+theta2)
    matrix[1,1] = D2*np.cos(theta1+theta2)
    return matrix

if __name__ == "__main__":
    t1 = 0.3
    t2 = -0.1
    d1 = -0.003
    d2 = 0.002

    print(fk(t1,t2))
    print(fk(t1+d1,t2+d2))
    print(jacobian(t1,t2))
    print(fk(t1,t2) + np.dot(jacobian(t1,t2),np.array([d1,d2])))
    print(fk(t1,t2) + np.dot(jacobian(t1,t2),np.array([d1,d2]))-fk(t1+d1,t2+d2))
    print(fk(t1,t2) + np.dot(jacobian(t1,t2),np.array([d1,d2]))-fk(t1,t2))