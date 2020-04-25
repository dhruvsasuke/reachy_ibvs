#!/usr/bin/env python

import numpy as np
import math as m
l0=0
l1=0.03970
l2=0.05200
l3=0.25600
l4=0.09700
l5=0.12700
l6=0.03300

d1 = l0+l1
d2 = 0
d3 = -(l2+l3)
d4 = 0
d5 = -(l4+l5)
d6 = 0

a1 = 0
a2 = 0
a3 = 0
a4 = 0
a5 = 0
a6 = 0



def calc_jack(th1,th2,th3,th4,th5,th6):
    
    j11=m.cos(th1)*((28*m.cos(th2)*m.cos(th4))/125 - (77*m.cos(th2))/250 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125)

    j12=m.cos(th2)*((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 - (77*m.sin(th1)*m.sin(th2))/250 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125) - m.sin(th1)*m.sin(th2)*((28*m.cos(th2)*m.cos(th4))/125 - (77*m.cos(th2))/250 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125)
 
    j13=-(m.cos(th1)*m.cos(th3) + m.cos(th2)*m.sin(th1)*m.sin(th3))*((28*m.cos(th2)*m.cos(th4))/125 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125) - m.sin(th2)*m.sin(th3)*((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125)
 
    j14=(m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)) + m.cos(th4)*m.sin(th1)*m.sin(th2))*((28*m.cos(th2)*m.cos(th4))/125 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125) - ((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125)*(m.cos(th2)*m.cos(th4) + m.cos(th3)*m.sin(th2)*m.sin(th4))
 
    j15=0

    j16=0



    j21=m.sin(th1)*((28*m.cos(th2)*m.cos(th4))/125 - (77*m.cos(th2))/250 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125)

    j22=m.cos(th2)*((77*m.cos(th1)*m.sin(th2))/250 + (28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125) + m.cos(th1)*m.sin(th2)*((28*m.cos(th2)*m.cos(th4))/125 - (77*m.cos(th2))/250 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125)

    j23=- (m.cos(th3)*m.sin(th1) - m.cos(th1)*m.cos(th2)*m.sin(th3))*((28*m.cos(th2)*m.cos(th4))/125 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125) - m.sin(th2)*m.sin(th3)*((28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125)
 
    j24=(m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)) - m.cos(th1)*m.cos(th4)*m.sin(th2))*((28*m.cos(th2)*m.cos(th4))/125 + (28*m.cos(th3)*m.sin(th2)*m.sin(th4))/125) - ((28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125)*(m.cos(th2)*m.cos(th4) + m.cos(th3)*m.sin(th2)*m.sin(th4))
 
    j25=0

    j26=0

	
    j31=m.sin(th1)*((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 - (77*m.sin(th1)*m.sin(th2))/250 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125) - m.cos(th1)*((77*m.cos(th1)*m.sin(th2))/250 + (28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125)
 
    j32=m.sin(th1)*m.sin(th2)*((77*m.cos(th1)*m.sin(th2))/250 + (28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125) + m.cos(th1)*m.sin(th2)*((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 - (77*m.sin(th1)*m.sin(th2))/250 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125)
 
    j33=((28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125)*(m.cos(th1)*m.cos(th3) + m.cos(th2)*m.sin(th1)*m.sin(th3)) - ((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125)*(m.cos(th3)*m.sin(th1) - m.cos(th1)*m.cos(th2)*m.sin(th3))
 
    j34=(m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)) - m.cos(th1)*m.cos(th4)*m.sin(th2))*((28*m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)))/125 + (28*m.cos(th4)*m.sin(th1)*m.sin(th2))/125) - ((28*m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)))/125 - (28*m.cos(th1)*m.cos(th4)*m.sin(th2))/125)*(m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)) + m.cos(th4)*m.sin(th1)*m.sin(th2))
 
    j35=0

    j36=0


    j41=m.sin(th1)

    j42=m.cos(th1)*m.sin(th2)
 
    j43=m.cos(th1)*m.cos(th2)*m.sin(th3) - m.cos(th3)*m.sin(th1)

    j44=m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)) - m.cos(th1)*m.cos(th4)*m.sin(th2)
 
    j45=- m.sin(th5)*(m.cos(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)) + m.cos(th1)*m.sin(th2)*m.sin(th4)) - m.cos(th5)*(m.cos(th3)*m.sin(th1) - m.cos(th1)*m.cos(th2)*m.sin(th3))
 
    j46=m.cos(th6)*(m.sin(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)) - m.cos(th1)*m.cos(th4)*m.sin(th2)) + m.sin(th6)*(m.cos(th5)*(m.cos(th4)*(m.sin(th1)*m.sin(th3) + m.cos(th1)*m.cos(th2)*m.cos(th3)) + m.cos(th1)*m.sin(th2)*m.sin(th4)) - m.sin(th5)*(m.cos(th3)*m.sin(th1) - m.cos(th1)*m.cos(th2)*m.sin(th3)))

    j51=-m.cos(th1)

    j52=m.sin(th1)*m.sin(th2)
 
    j53=m.cos(th1)*m.cos(th3) + m.cos(th2)*m.sin(th1)*m.sin(th3)

    j54=- m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)) - m.cos(th4)*m.sin(th1)*m.sin(th2)
 
    j55=m.sin(th5)*(m.cos(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)) - m.sin(th1)*m.sin(th2)*m.sin(th4)) + m.cos(th5)*(m.cos(th1)*m.cos(th3) + m.cos(th2)*m.sin(th1)*m.sin(th3))

    j56=- m.cos(th6)*(m.sin(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)) + m.cos(th4)*m.sin(th1)*m.sin(th2)) - m.sin(th6)*(m.cos(th5)*(m.cos(th4)*(m.cos(th1)*m.sin(th3) - m.cos(th2)*m.cos(th3)*m.sin(th1)) - m.sin(th1)*m.sin(th2)*m.sin(th4)) - m.sin(th5)*(m.cos(th1)*m.cos(th3) + m.cos(th2)*m.sin(th1)*m.sin(th3)))


    j61=0

    j62=-m.cos(th2)

    j63=m.sin(th2)*m.sin(th3)

    j64=m.cos(th2)*m.cos(th4) + m.cos(th3)*m.sin(th2)*m.sin(th4)

    j65=m.sin(th5)*(m.cos(th2)*m.sin(th4) - m.cos(th3)*m.cos(th4)*m.sin(th2)) + m.cos(th5)*m.sin(th2)*m.sin(th3)

    j66=m.cos(th6)*(m.cos(th2)*m.cos(th4) + m.cos(th3)*m.sin(th2)*m.sin(th4)) - m.sin(th6)*(m.cos(th5)*(m.cos(th2)*m.sin(th4) - m.cos(th3)*m.cos(th4)*m.sin(th2)) - m.sin(th2)*m.sin(th3)*m.sin(th5))
	
	
    jacobian = [[j11,j12,j13,j14,j15,j16],[j21,j22,j23,j24,j25,j26],[j31,j32,j33,j34,j35,j36],[j41,j42,j43,j44,j45,j46],[j51,j52,j53,j54,j55,j56],[j61,j62,j63,j64,j65,j66]]	
	
    jacobian = np.linalg.pinv(jacobian)


    return jacobian

print(calc_jack(0,0,0,0,0,0))