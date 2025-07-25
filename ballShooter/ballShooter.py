
import wpilib
from utils.constants import BALL_SHOOTER_OUT_LEFT, BALL_SHOOTER_OUT_RIGHT
from utils.signalLogging import addLog
from utils.singleton import Singleton


class BallShooterControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommandState = False
        self.ejectCommandState = False
        self.shooterRMotor = wpilib.Spark(BALL_SHOOTER_OUT_LEFT)
        self.shooterLMotor = wpilib.Spark(BALL_SHOOTER_OUT_RIGHT)

        addLog("Algae Manipulator intake cmd",lambda:self.intakeCommandState,"Bool")
        addLog("Algae Manipulator  cmd",lambda:self.ejectCommandState,"Bool")

    def update(self):

        if self.intakeCommandState:
            self.updateIntake(True)
        elif self.ejectCommandState:
            self.updateEject(True)
        else:
            self.shooterLMotor.set(-.05)
            self.shooterRMotor.set(.05)

    def setInput(self, intakeBool, ejectBool):
        self.intakeCommandState = intakeBool
        self.ejectCommandState = ejectBool

    def updateIntake(self, run, ):
        voltage = -.8

        if run:
            self.shooterRMotor.set(voltage)
            self.shooterLMotor.set(-voltage)
        else: 
            self.shooterRMotor.set(0)
            self.shooterLMotor.set(0)
    
    def updateEject(self, run):
        voltage = .8

        if run:
            self.shooterRMotor.set(voltage)
            self.shooterLMotor.set(-voltage)
        else:
            self.shooterLMotor.set(0)
            self.shooterRMotor.set(0)