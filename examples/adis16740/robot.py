#!/usr/bin/env python3

import wpilib
from wpilib import SendableChooser
from wpilib import SmartDashboard
from adis16470 import ADIS16470_IMU
from adis16470 import ADIS16470CalibrationTime

KAUTONAME_DEFAULT = "Default"
KAUTONAME_CUSTOM = "My Auto"
KYAW_DEFAULT = "Z-Axis"
KYAW_X_AXIS = "X-Axis"
KYAW_Y_AXIS = "Y-Axis"

def run():
    raise ValueError()


class MyRobot(wpilib.TimedRobot):
    m_autoSelected = ""

    def robotInit(self):

        self.timer = wpilib.Timer()

        self.m_imu = ADIS16470_IMU()
        self.m_yawSelected = KYAW_DEFAULT
        self.m_runCal = False
        self.m_configCal = False
        self.m_reset = False
        self.m_setYawAxis = False
        
        self.m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kZ

        self.m_autoChooser = wpilib.SendableChooser()
        self.m_autoChooser.AddOption(KAUTONAME_CUSTOM, KAUTONAME_CUSTOM)

        self.m_yawChooser = wpilib.SendableChooser()
        self.m_yawChooser.SetDefaultOption(KYAW_DEFAULT, KYAW_DEFAULT)
        self.m_yawChooser.AddOption(kYawXAxis, kYawXAxis)
        self.m_yawChooser.AddOption(kYawYAxis, kYawYAxis)
        
        wpilib.SmartDashboard.putData("Auto Modes", m_autoChooser)
        wpilib.SmartDashboard.putData("IMUYawAxis", m_yawChooser)

        wpilib.SmartDashboard.putBoolean("RunCal", False)
        wpilib.SmartDashboard.putBoolean("ConfigCal", False)
        wpilib.SmartDashboard.putBoolean("Reset", False)
        wpilib.SmartDashboard.putBoolean("SetYawAxis", False)

    
    '''
    * This function is called every robot packet, no matter the mode. Use
    * this for items like diagnostics that you want ran during disabled,
    * autonomous, teleoperated and test.
    *
    * This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    '''
 
    def robotPeriodic(self) :
        wpilib.SmartDashboard.putNumber("YawAngle", self.m_imu.getAngle())
        wpilib.SmartDashboard.putNumber("XCompAngle", self.m_imu.getXComplementaryAngle())
        wpilib.SmartDashboard.putNumber("YCompAngle", self.m_imu.getYComplementaryAngle())
        wpilib.SmartDashboard.getBoolean("RunCal", False)
        wpilib.SmartDashboard.getBoolean("RunCal", False)
        wpilib.SmartDashboard.getBoolean("RunCal", False)
        wpilib.SmartDashboard.getBoolean("RunCal", False)
        self.m_yawSelected = self.m_yawChooser.GetSelected()
        self.m_yawSelected = KYAW_DEFAULT  

        # Set IMU settings
        if (self.m_configCal) :
            self.m_imu.configCalTime(ADIS16470CalibrationTime._8s)
            self.m_configCal = SmartDashboard.putBoolean("ConfigCal", False)


        if (self.m_reset) :
            self.m_imu.Reset()
            self.m_reset = SmartDashboard.putBoolean("Reset", False)

        if (self.m_runCal) :
            self.m_imu.Calibrate()
            self.m_runCal = SmartDashboard.putBoolean("RunCal", False)

        # Read the desired yaw axis from the dashboard
        if (self.m_yawSelected == "X-Axis") :
            self.m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kX
        
        elif (self.m_yawSelected == "Y-Axis") :
            self.m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kY

        else:
            self.m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kZ 

        # Set the desired yaw axis from the dashboard
        if (self.m_setYawAxis) :
            self.m_imu.SetYawAxis(self.m_yawActiveAxis)
            self.m_setYawAxis = SmartDashboard.putBoolean("SetYawAxis", False)


    # '''
    # * This autonomous (along with the chooser code above) shows how to select
    # * between different autonomous modes using the dashboard. The sendable chooser
    # * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
    # * remove all of the chooser code and uncomment the GetString line to get the
    # * auto name from the text box below the Gyro.
    # *
    # * You can add additional auto modes by adding additional comparisons to the
    # * if-else structure below with additional strings. If using the SendableChooser
    # * make sure to add them to the chooser code above as well.
    # '''

    def autonomousInit(self):
        #self.m_autoSelected = self.m_autoChooser.GetSelected()
        self.m_autoSelected = KAUTONAME_CUSTOM
        SmartDashboard.putString("Auto Selected", self.m_autoSelected)
        print("Auto selected: " + self.m_autoSelected)

        if (self.m_autoSelected == KAUTONAME_CUSTOM) :
            # Custom Auto goes here
            print("Auto selected: " + "Custom Autonomous")            
        else :
            # Default Auto goes here
            print("Auto selected: " + "Default Autonomous")

    def autonomousPeriodic(self):
        self.m_autoSelected = KAUTONAME_CUSTOM

        if (self.m_autoSelected == KAUTONAME_CUSTOM) :
            # Custom Auto goes here
            print("Auto selected: " + "Custom Autonomous")            
        else :
            # Default Auto goes here
            print("Auto selected: " + "Default Autonomous")

        SmartDashboard.putString("Auto Selected", self.m_autoSelected)

        wpilib.SmartDashboard.putNumber("YawAngle", self.m_imu.getAngle())



    def disabledInit(self):
        print("Entered Disabled Init")

    def disabled(self):
        print("Entered Disabled method")
    #     sendableImuData = wpilib.SendableBuilder()
    #     self.m_imu.InitSendable(sendableImuData)

    #     self.logger.info("Entered disabled mode")

        self.timer.reset()
        self.timer.start()

        while self.isDisabled():

            if self.timer.hasPeriodPassed(0.5):
                print("Disabled status loop.")
                SmartDashboard.putNumber("YawAngle", self.m_imu.GetAngle())
                SmartDashboard.putNumber("XCompAngle", self.m_imu.GetXComplementaryAngle())
                SmartDashboard.putNumber("XFilteredAccelAngle", self.m_imu.GetXFilteredAccelAngle())                
                SmartDashboard.putNumber("YCompAngle", self.m_imu.GetYComplementaryAngle())
                SmartDashboard.putNumber("YFilteredAccelAngle", self.m_imu.GetYFilteredAccelAngle())

                SmartDashboard.putNumber("AccelInstantX", self.m_imu.GetAccelInstantX())
                SmartDashboard.putNumber("AccelInstantY", self.m_imu.GetAccelInstantY())
                SmartDashboard.putNumber("AccelInstantZ", self.m_imu.GetAccelInstantZ())

                SmartDashboard.putNumber("GyroInstantX", self.m_imu.GetGyroInstantX())
                SmartDashboard.putNumber("GyroInstantY", self.m_imu.GetGyroInstantY())
                SmartDashboard.putNumber("GyroInstantZ", self.m_imu.GetGyroInstantZ())
                SmartDashboard.putNumber("GyroRate", self.m_imu.GetRate())
                
                SmartDashboard.putNumber("GyroRate", self.m_imu.GetRate())
                self.timer.reset()

           ## Do not use in RobotPy - wpilib.Timer.delay(0.10)



if __name__ == "__main__":
    wpilib.run(MyRobot)
















# class Robot : public frc::TimedRobot {
#  public:
#   void RobotInit() override;
#   void RobotPeriodic() override;
#   void AutonomousInit() override;
#   void AutonomousPeriodic() override;
#   void TeleopInit() override;
#   void TeleopPeriodic() override;
#   void TestPeriodic() override;

#  private:
#   frc::SendableChooser<std::string> m_autoChooser;
#   const std::string kAutoNameDefault = "Default";
#   const std::string kAutoNameCustom = "My Auto";
#   std::string m_autoSelected;
#   frc::ADIS16470_IMU m_imu{};
#   frc::SendableChooser<std::string> m_yawChooser;
#   const std::string kYawDefault = "Z-Axis";
#   const std::string kYawXAxis = "X-Axis";
#   const std::string kYawYAxis = "Y-Axis";
#   std::string m_yawSelected;
#   bool m_runCal = false;
#   bool m_configCal = false;
#   bool m_reset = false;
#   bool m_setYawAxis = false;
#   frc::ADIS16470_IMU::IMUAxis m_yawActiveAxis = frc::ADIS16470_IMU::IMUAxis::kZ;
# };

# void Robot::RobotInit() {
#   m_autoChooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
#   m_autoChooser.AddOption(kAutoNameCustom, kAutoNameCustom);
#   m_yawChooser.SetDefaultOption(kYawDefault, kYawDefault);
#   m_yawChooser.AddOption(kYawXAxis, kYawXAxis);
#   m_yawChooser.AddOption(kYawYAxis, kYawYAxis);
#   SmartDashboard::PutData("Auto Modes", m_autoChooser);
#   frc::SmartDashboard::PutData("IMUYawAxis", m_yawChooser);
#   frc::SmartDashboard::PutBoolean("RunCal", false);
#   frc::SmartDashboard::PutBoolean("ConfigCal", false);
#   frc::SmartDashboard::PutBoolean("Reset", false);
#   frc::SmartDashboard::PutBoolean("SetYawAxis", false);
# }

# /**
#  * This function is called every robot packet, no matter the mode. Use
#  * this for items like diagnostics that you want ran during disabled,
#  * autonomous, teleoperated and test.
#  *
#  * <p> This runs after the mode specific periodic functiosns, but before
#  * LiveWindow and SmartDashboard integrated updating.
#  */
# void Robot::RobotPeriodic() {
#   frc::SmartDashboard::PutNumber("YawAngle", m_imu.GetAngle());
#   frc::SmartDashboard::PutNumber("XCompAngle", m_imu.GetXComplementaryAngle());
#   frc::SmartDashboard::PutNumber("YCompAngle", m_imu.GetYComplementaryAngle());
#   m_runCal = frc::SmartDashboard::GetBoolean("RunCal", false);
#   m_configCal = frc::SmartDashboard::GetBoolean("ConfigCal", false);
#   m_reset = frc::SmartDashboard::GetBoolean("Reset", false);
#   m_setYawAxis = frc::SmartDashboard::GetBoolean("SetYawAxis", false);
#   m_yawSelected = m_yawChooser.GetSelected();

#   // Set IMU settings
#   if (m_configCal) {
#     m_imu.ConfigCalTime(frc::ADIS16470CalibrationTime::_8s);
#     m_configCal = frc::SmartDashboard::PutBoolean("ConfigCal", false);
#   }
#   if (m_reset) {
#     m_imu.Reset();
#     m_reset = frc::SmartDashboard::PutBoolean("Reset", false);
#   }
#   if (m_runCal) {
#     m_imu.Calibrate();
#     m_runCal = frc::SmartDashboard::PutBoolean("RunCal", false);
#   }
  
#   // Read the desired yaw axis from the dashboard
#   if (m_yawSelected == "X-Axis") {
#     m_yawActiveAxis = frc::ADIS16470_IMU::IMUAxis::kX;
#   }
#   else if (m_yawSelected == "Y-Axis") {
#     m_yawActiveAxis = frc::ADIS16470_IMU::IMUAxis::kY;
#   }
#   else {
#     m_yawActiveAxis = frc::ADIS16470_IMU::IMUAxis::kZ;
#   }
#   // Set the desired yaw axis from the dashboard
#   if (m_setYawAxis) {
#     m_imu.SetYawAxis(m_yawActiveAxis);
#     m_setYawAxis = frc::SmartDashboard::PutBoolean("SetYawAxis", false);
#   }
# }

# /**
#  * This autonomous (along with the chooser code above) shows how to select
#  * between different autonomous modes using the dashboard. The sendable chooser
#  * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
#  * remove all of the chooser code and uncomment the GetString line to get the
#  * auto name from the text box below the Gyro.
#  *
#  * You can add additional auto modes by adding additional comparisons to the
#  * if-else structure below with additional strings. If using the SendableChooser
#  * make sure to add them to the chooser code above as well.
#  */
# void Robot::AutonomousInit() {
#   m_autoSelected = m_autoChooser.GetSelected();
#   // m_autoSelected = SmartDashboard::GetString("Auto Selector",
#   //     kAutoNameDefault);
#   std::cout << "Auto selected: " << m_autoSelected << std::endl;

#   if (m_autoSelected == kAutoNameCustom) {
#     // Custom Auto goes here
#   } else {
#     // Default Auto goes here
#   }
# }

# void Robot::AutonomousPeriodic() {
#   if (m_autoSelected == kAutoNameCustom) {
#     // Custom Auto goes here
#   } else {
#     // Default Auto goes here
#   }
# }

# void Robot::TeleopInit() {}

# void Robot::TeleopPeriodic() {}

# void Robot::TestPeriodic() {}

# #ifndef RUNNING_FRC_TESTS
# int main() { return frc::StartRobot<Robot>(); }
# #endif