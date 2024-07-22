import legoM
import numpy as np
def main():
    # This is a main function
    run = legoM.LegoM()
    run.DH()
    run.IK([100,50,10,np.pi,np.pi/2,np.pi/2],'radians')
main()    
