import sys, subprocess,roboticstoolbox as rtb, numpy as np
from spatialmath import SE3
import keyboard
class LegoM:
    def __init__(self):
        self.fileName = "main.py"
        self.limitsDegrees = [[-170, 170], [-350, 350], [10, 155], [-115, 115], [-150, -10], [-350, 350]]
        self.limitsRadian = np.deg2rad(self.limitsDegrees)
        self.__DHParams()
    def __DHParams(self):
        self.dh_params = [
            [-np.pi/2, 16, 132, 0],                           
            [0,112, 0, 0],            
            [-np.pi/2, 40, 0, -np.pi/2],                    
            [np.pi/2, 0, 160,0],         
            [-np.pi/2, 0, 0, 0],                    
            [0, 0, 44,np.pi]                 
        ]
        self.links = []
        for param in self.dh_params:
            self.link = rtb.RevoluteDH(d=param[2], a=param[1], alpha=param[0], offset=param[3])
            self.links.append(self.link)
        self.kuka_robot = rtb.DHRobot(self.links, name='KUKA 6-DOF')
        limitArray = np.ndarray(shape=(2,6),dtype=float, order='F', buffer=self.limitsRadian)
        print(limitArray)
        print(self.limitsRadian)
        self.kuka_robot.qlim = limitArray
    def DH(self):
        print(self.kuka_robot)
    def FK(self,angles,type):
        if len(angles) != 6:
            print('Please provide 6 joint angles')
            return
        self.limits = self.limitsRadian
        if type == 'degrees' or type == 'deg':
            angles = np.deg2rad(angles)
        for i in range(6):
            if angles[i] < self.limits[i][0] or angles[i] > self.limits[i][1]:
                print(f'Joint {i+1} is out of limits')
                return
        self.solution = self.kuka_robot.fkine(angles)
        print(f'End-effector pose: {self.solution}')
        angles = angles.tolist()
        self.writeFile(angles)
        
          
    def IK(self,pose,type):
        if len(pose) != 6:
            print('Please provide 6 pose values')
            return
        if type == 'degrees' or type == 'deg':
            pose[3] = np.deg2rad(pose[3])
            pose[4] = np.deg2rad(pose[4])
            pose[5] = np.deg2rad(pose[5])
        T = SE3.Trans(pose[0], pose[1], pose[2]) * SE3.RPY(pose[3], pose[4], pose[5])
        self.solution = self.kuka_robot.ikine_GN(T,joint_limits=True)
        if self.solution.success:
           print(f'Solution found: {self.solution.q}')
        else:
            print('No solutions found for the given pose.')
            return
        
        self.sol_lists = [0] * len(self.solution.q)
        self.sol_lists = self.solution.q * 180 / np.pi
        self.sol_lists = self.sol_lists.tolist()
        self.writeFile(self.sol_lists)
    def writeFile(self, sols):

        content = (
    "from pybricks.hubs import InventorHub\n"
    "from pybricks.pupdevices import Motor\n"
    "from pybricks.parameters import Port, Direction, Stop\n"
    "from pybricks.tools import wait, StopWatch, multitask, run_task\n\n"
    
    "hub = InventorHub()\n"
    "motorA = Motor(Port.A, reset_angle=False, gears=[1,37])\n"
    "motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[1,21])\n"
    "motorC = Motor(Port.C, reset_angle=False, gears=[1,52])\n"
    "motorD = Motor(Port.D, reset_angle=False, gears=[1,22])\n"
    "motorE = Motor(Port.E, Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[1,67])\n"
    "motorF = Motor(Port.F, Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[1,12])\n"
    f"jointLimits = {self.limitsDegrees}\n"
    f"direction_list = {sols}\n"
    "motorSpeed = 500\n"
    "torqueLimit = 100\n"
    "torqueLimitWrist = 30\n\n"
    
    "def dutyLimitMotors():\n"
    "    motorA.control.limits(torque=100)\n"
    "    motorB.control.limits(torque=135)\n"
    "    motorC.control.limits(torque=torqueLimit)\n"
    "    motorD.control.limits(torque=torqueLimit)\n"
    "    motorE.control.limits(torque=torqueLimit)\n"
    "    motorF.control.limits(torque=torqueLimit)\n\n"
    
    "async def homingMotors():\n"
    "    await multitask(\n"
    "        motorA.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=30),\n"
    "        motorB.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=15),\n"
    "        motorC.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=35),\n"
    "        motorD.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=35),\n"
    "        motorE.run_until_stalled(-motorSpeed, then=Stop.COAST_SMART, duty_limit=35)\n"
    "    )\n\n"
    
    "def setHomingLimits():\n"
    "    for motor, limit in zip([motorA, motorB, motorC, motorD, motorE, motorF], jointLimits):\n"
    "        motor.reset_angle(limit[0])\n\n"
    
    "def homing():\n"
    "    run_task(homingMotors())\n"
    "    setHomingLimits()\n"
    "    return True\n\n"
    
    "async def check_motor():\n"
    "    return all(motor.done() for motor in [motorA, motorB, motorC, motorD, motorE, motorF])\n\n"
    
    "def calcDirection(dir, motor):\n"
    "    angleCalc = motor.angle()\n"
    "    if dir > 0:\n"
    "        if angleCalc < 0:\n"
    "            angleCalc = abs(angleCalc) + dir\n"
    "        else:\n"
    "            angleCalc = dir - angleCalc\n"
    "    else:\n"
    "        if angleCalc < 0:\n"
    "            angleCalc = abs(angleCalc) + dir\n"
    "        else:\n"
    "            angleCalc = dir - angleCalc\n"
    "    return angleCalc\n\n"
    "async def stallMotors():\n"
    "    if motorA.stalled():\n"
    "        motorA.stop()\n"
    "        print('Motor A stalled at',motorA.angle())\n"
    "    if motorB.stalled():\n"
    "        motorB.stop()\n"
    "        print('Motor B stalled at',motorB.angle())\n"
    "    if motorC.stalled():\n"
    "        motorC.stop()\n"
    "        print('Motor C stalled at',motorC.angle())\n"
    "    if motorD.stalled():\n"
    "        motorD.stop()\n"
    "        print('Motor D stalled at',motorD.angle())\n"
    "    if motorE.stalled():\n"
    "        motorE.stop()\n"
    "        print('Motor E stalled at',motorE.angle())\n"
    "    if motorF.stalled():\n"
    "        motorF.stop()\n\n"
    "        print('Motor F stalled at',motorF.angle())\n\n"
        
    "async def run_motors():\n"
    "    await multitask(\n"
    "        motorA.run_angle(motorSpeed, calcDirection(direction_list[0], motorA), Stop.COAST_SMART),\n"
    "        motorF.run_angle(motorSpeed, calcDirection(direction_list[5], motorF), Stop.COAST_SMART),\n"
    "        motorB.run_angle(motorSpeed, calcDirection(direction_list[1], motorB), Stop.COAST_SMART),\n"
    "        motorD.run_angle(motorSpeed, calcDirection(direction_list[3], motorD), Stop.COAST_SMART),\n"
    "        motorC.run_angle(motorSpeed, calcDirection(direction_list[2], motorC), Stop.COAST_SMART),\n"
    "        motorE.run_angle(motorSpeed, calcDirection(direction_list[4], motorE), Stop.COAST_SMART)\n"
    "    )\n\n"
    
    "async def print_angles(watch):\n"
    "    print(f\"{'Motor':>10} {'A':>10} {'B':>10} {'C':>10} {'D':>10} {'E':>10} {'F':>10}\")\n"
    "    print(f\"{'Target':>10} {direction_list[0]:>10.2f} {direction_list[1]:>10.2f} {direction_list[2]:>10.2f} {direction_list[3]:>10.2f} {direction_list[4]:>10.2f} {direction_list[5]:>10.2f}\")\n"
    "    while True:\n"
    "        angles = [\n"
    "            motorA.angle(),\n"
    "            motorB.angle(),\n"
    "            motorC.angle(),\n"
    "            motorD.angle(),\n"
    "            motorE.angle(),\n"
    "            motorF.angle()\n"
    "        ]\n"
    "        if await check_motor():\n"
    "            watch.stop()\n"
    "            print('Jobs done')\n"
    "            print('Elapsed time:',watch.time()/1000,'s')\n"
    "            print(f\"{'Error':>10} {abs(direction_list[0]) - abs(angles[0]):>10.2f} {abs(direction_list[1]) - abs(angles[1]):>10.2f} {abs(direction_list[2]) - abs(angles[2]):>10.2f} {abs(direction_list[3]) - abs(angles[3]):>10.2f} {abs(direction_list[4]) - abs(angles[4]):>10.2f} {abs(direction_list[5]) - abs(angles[5]):>10.2f}\")\n"
    "            print(f\"{'Angle result':>10} {angles[0]:>10.2f} {angles[1]:>10.2f} {angles[2]:>10.2f} {angles[3]:>10.2f} {angles[4]:>10.2f} {angles[5]:>10.2f}\")\n"
    "            break\n\n"
    "        await wait(100)\n\n"
    
    "async def driveMotors(watch):\n"
    "    await multitask(run_motors(),stallMotors(), print_angles(watch))\n\n"
    
    "def main():\n"
    "    homing()\n"
    "    dutyLimitMotors()\n"
    "    watch = StopWatch()\n"
    "    run_task(driveMotors(watch))\n"
    "    watch.reset()\n\n"
    
    "main()\n"
        )
        print("Writing file...")
        self.__createFile(content)
    def limits(self):
        print(self.limitsDegrees)

    def __terminalCmd(self, command):
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        print(result.stdout)
        return result
    def __createFile(self,content):
        with open(self.fileName, 'w') as file:
            file.write(content)
        command = 'mpy-cross ' + self.fileName
        result = self.__terminalCmd(command)
        if result.returncode != 0:
            print("Failed to compile the file")
            sys.exit(1)
        self.__runFile()     
    def __runFile(self):
        print("Press 'q' to run the program, make sure bluetooth on your device is on and the robot is turned on")
        keyboard.wait('q')
        command = f'pipx run pybricksdev run ble main.mpy'
        result = self.__terminalCmd(command)