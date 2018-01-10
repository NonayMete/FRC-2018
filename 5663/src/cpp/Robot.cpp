#include "WPILib.h"
#include "AHRS.h"
#include "curtinfrc/math.h"
#include <iostream>
#include <string>
#include <SmartDashboard/SmartDashboard.h>
#include <PowerDistributionPanel.h>
#include "ctre/Phoenix.h"
#include <PIDOutput.h>
<<<<<<< HEAD
=======

const static double kP = 1.0f;
const static double kI = 0.00f;
const static double kD = 0.00f;
const static double kF = 0.00f;
>>>>>>> Jake/Auto

using namespace frc;
using namespace curtinfrc;
using namespace std;

class gyroPID : public PIDOutput {
private:
  double thisisoutput = 0;

public:
  void PIDWrite(double output) {
    thisisoutput = output;
  }
  double GetOutput() {
    return thisisoutput;
  }
};

class Robot : public IterativeRobot {
  XboxController *xbox;
  Victor *left2= new Victor(6), *left1 = new Victor(7), *right1 = new Victor(8), *right2 = new Victor(9),
			*gear_motor = new Victor(4);
  AHRS *ahrs;
  PIDController *turn;
<<<<<<< HEAD
=======
  gyroPID *out;
>>>>>>> Jake/Auto
public:
  double deadzone = 0.04;
  string gameData;
  int Auto, gearMode;
  void RobotInit() {
    xbox = new XboxController(0);
<<<<<<< HEAD
    pdp = new PowerDistributionPanel(0);
    AutoChooser = new SendableChooser<int*>;
    left1 = new TalonSRX(1); left2 = new TalonSRX(2); left3 = new TalonSRX(3);
    right1 = new TalonSRX(4); right2 = new TalonSRX(4); right3 = new TalonSRX(5);
    leftGear = new DoubleSolenoid(0,0,1); rightGear = new DoubleSolenoid(0,2,3);
    AutoChooser->AddDefault("Cross Baseline",(int*) 0);
    AutoChooser->AddObject("Auto 1",(int*) 1);
    AutoChooser->AddObject("Auto 2",(int*) 2);
=======
    out = new gyroPID();
    ahrs = new AHRS(I2C::Port::kMXP);
    ahrs->EnableLogging(true);
    turn = new PIDController(kP,kI,kD,kF,ahrs,this);
    turn->SetInputRange(-180.0, 180.0);
    turn->SetOutputRange(-1.0,1.0);
    turn->SetAbsoluteTolerance(2.0);
    turn->SetContinuous(true);
    getValues();
    updateDash();
>>>>>>> Jake/Auto
  }

  void Drive(double l, double r) {
    // if(-deadzone < l && l < deadzone) l == 0;
    // if(-deadzone < r && r < deadzone) r == 0;
    l *= abs(l);
    r *= abs(r); // square inputs
<<<<<<< HEAD
     left1->Set(ControlMode::PercentOutput,l); left2->Set(ControlMode::PercentOutput,l); left3->Set(ControlMode::PercentOutput,l);
     right1->Set(ControlMode::PercentOutput,l); right2->Set(ControlMode::PercentOutput,l); right2->Set(ControlMode::PercentOutput,l);
=======
    left1->Set(l); left2->Set(l);
    right1->Set(r); right2->Set(r);
  }

  void getValues() { //Set variables for the xbox controller values for easy coding (put in seperate file?)
    A = xbox->GetRawButton(1);
    B = xbox->GetRawButton(2);
    X = xbox->GetRawButton(3);
    Y = xbox->GetRawButton(4);
    LB = xbox->GetRawButton(5);
    RB = xbox->GetRawButton(6);
    back = xbox->GetRawButton(7);
    start = xbox->GetRawButton(8);
    LS = xbox->GetRawButton(9);
    RS = xbox->GetRawButton(10);
    LX = xbox->GetRawAxis(0);
    LY = xbox->GetRawAxis(1);
    LT = xbox->GetRawAxis(2);
    RT = xbox->GetRawAxis(3);
    RX = xbox->GetRawAxis(4);
    RY = xbox->GetRawAxis(5);
    Dpad = xbox->GetPOV();
  }

  void updateDash() { //Put new controller values to the dashboard (put in seperate file?)
    SmartDashboard::PutNumber("D-Pad", Dpad);
    SmartDashboard::PutBoolean("A", A);
    SmartDashboard::PutBoolean("B", B);
    SmartDashboard::PutBoolean("X", X);
  	SmartDashboard::PutBoolean("Y", Y);
  	SmartDashboard::PutBoolean("Left Bumper", LB);
    SmartDashboard::PutBoolean("Right Bumper", RB);
    SmartDashboard::PutBoolean("Back", back);
    SmartDashboard::PutBoolean("Start", start);
    SmartDashboard::PutBoolean("Left Stick", LS);
  	SmartDashboard::PutBoolean("Right Stick", RS);
    SmartDashboard::PutNumber("Left Stick X", LX);
    SmartDashboard::PutNumber("Right Stick X", RX);
    SmartDashboard::PutNumber("Left Stick Y", LY);
    SmartDashboard::PutNumber("Right Stick Y", RY);
    SmartDashboard::PutNumber("Left Trigger", LT);
    SmartDashboard::PutNumber("Right Trigger", RT);
>>>>>>> Jake/Auto
  }

  void AutonomousInit() {
    gameData = DriverStation::GetInstance().GetGameSpecificMessage(); //Get specific match data
    SmartDashboard::PutString("Alliance Switch:", &gameData[0]);
    SmartDashboard::PutString("Scale:", &gameData[1]);
    SmartDashboard::PutString("Enemy Switch:", &gameData[2]);  //Put data on shuffleboard
  }
  void AutonomousPeriodic() {
    // gameData will be an array with 3 characters, eg. "LRL"
    // check https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details
  }

  void TeleopInit() {
    turn->SetSetpoint(ahrs->GetAngle());
    turn->Enable();
  }
  void TeleopPeriodic() {
<<<<<<< HEAD
    Drive(xbox->GetY(xbox->kLeftHand),xbox->GetY(xbox->kRightHand));
    if(xbox->GetBumperPressed(xbox->kLeftHand) == true) {
      if(rightGear->Get() == rightGear->kForward) gearMode = rightGear->kReverse;
      else gearMode = rightGear->kForward;
    }
    if(gearMode == rightGear->kForward) {
      rightGear->Set(rightGear->kForward);
      leftGear->Set(rightGear->kForward);
    } else {
      rightGear->Set(rightGear->kReverse);
      leftGear->Set(rightGear->kReverse);
    }
  }

  void TestInit() {

=======
    getValues();
    updateDash();
    if(A) {
      turn->SetSetpoint(90.0f);
    }
    if(B) {
      turn->SetSetpoint(-90.0f);
    }
    Drive(out->GetOutput(),-out->GetOutput());
    SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
    SmartDashboard::PutBoolean("IMU_IsMoving", ahrs->IsMoving());
    SmartDashboard::PutBoolean("IMU_IsCalibrating", ahrs->IsCalibrating());
    SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
    SmartDashboard::PutNumber("Setpoint", turn->GetSetpoint());
    SmartDashboard::PutNumber("PID output" , turn->Get());
>>>>>>> Jake/Auto
  }

  void TestPeriodic() {
  }
};

START_ROBOT_CLASS(Robot)
