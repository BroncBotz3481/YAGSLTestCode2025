// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMath.Arm;
import frc.robot.RobotMath.Elevator;

public class Constants
{

  public static final int kMotorPort       = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort    = 0;

  public static final Mechanism2d         sideRobotView = new Mechanism2d(ArmConstants.kArmLength * 2,
                                                                          ElevatorConstants.kMaxElevatorHeight.in(
                                                                              Meters) +
                                                                          ArmConstants.kArmLength);
  public static final MechanismRoot2d     kElevatorCarriage;
  public static final MechanismLigament2d kArmMech;
  public static final MechanismLigament2d kElevatorTower;

  static
  {
    kElevatorCarriage = Constants.sideRobotView.getRoot("ElevatorCarriage",
                                                        ArmConstants.kArmLength,
                                                        ElevatorConstants.kStartingHeightSim.in(
                                                    Meters));
    kArmMech = kElevatorCarriage.append(
        new MechanismLigament2d(
            "Arm",
            ArmConstants.kArmLength,
            ArmConstants.kArmStartingAngle.in(Degrees),
            6,
            new Color8Bit(Color.kYellow)));
    kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
        "Elevator",
        ElevatorConstants.kStartingHeightSim.in(Meters),
        -90,
        6,
        new Color8Bit(Color.kRed)));
  }

  public static class ArmConstants
  {

    // The P gain for the PID controller that drives this arm.
    public static final double kArmKp                     = 2.0691;
    public static final double kArmKi                     = 0;
    public static final double kArmKd                     = 0.0;
    public static final Angle  kArmAllowedClosedLoopError = Arm.convertAngleToSensorUnits(Degrees.of(0.01));

    public static final double  kArmReduction                   = 200;
    public static final double  kArmMass                        = 8.0; // Kilograms
    public static final double  kArmLength                      = Inches.of(72).in(Meters);
    public static final Angle   kArmStartingAngle               = Degrees.of(0);
    public static final Angle   kMinAngle                       = Degrees.of(-75);
    public static final Angle   kMaxAngle                       = Degrees.of(255);
    public static final double  kArmRampRate                    = 0.5;
    public static final Angle   kArmOffsetToHorizantalZero      = Rotations.of(0);
    public static final boolean kArmInverted                    = false;
    public static final double  kArmMaxVelocityRPM              = Arm.convertAngleToSensorUnits(Degrees.of(90)).per(
        Second).in(RPM);
    public static final double  kArmMaxAccelerationRPMperSecond = Arm.convertAngleToSensorUnits(Degrees.of(180)).per(
                                                                         Second).per(Second)
                                                                     .in(RPM.per(Second));
    public static final int     kArmStallCurrentLimitAmps       = 40;

    public static final double kArmkS = 0; // volts (V)
    public static final double kArmkG = 0; // volts (V)
    public static final double kArmKv = 0; // volts per velocity (V/RPM)
    public static final double kArmKa = 0; // volts per acceleration (V/(RPM/s))


  }

  public static class ElevatorConstants
  {


    public static final double kElevatorKp = 26.722;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 1.6047;

    public static final double kElevatorkS = 0.01964; // volts (V)
    public static final double kElevatorkV = 3.894; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.173; // volt per acceleration (V/(m/s²))
    public static final double kElevatorkG = 0.91274; // volts (V)

    public static final double kElevatorGearing    = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass       = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kLaserCANOffset    = Inches.of(3);
    public static final Distance kStartingHeightSim = Meters.of(0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(10.25);


    public static double kElevatorRampRate = 0.1;
    public static int    kElevatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
  }


}