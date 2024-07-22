import legoM
import numpy as np
def main():
    # This is a main function
    run = legoM.LegoM()
    run.DH()
    run.IK([125,150,60,np.pi/4,np.pi/8,np.pi],'radians')
main()    
