// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class Constants
{

  public static final int kMotorPort       = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort    = 0;

  public static class ArmConstants
  {

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey        = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp             = 2.0691;
    public static final double kArmKi                    = 0;
    public static final double kArmKd                    = 0.0;
    public static final Angle kArmAllowedClosedLoopError = Degrees.of(0.01);

    public static final double  kArmReduction                   = 200;
    public static final double  kArmMass                        = 8.0; // Kilograms
    public static final double  kArmLength                      = Units.inchesToMeters(30);
    public static final Angle   kMinAngleRads                   = Radians.of(Units.degreesToRadians(-75));
    public static final Angle   kMaxAngleRads                   = Radians.of(Units.degreesToRadians(255));
    public static final double  kArmRampRate                    = 0.5;
    public static final Angle   kArmOffsetToHorizantalZero      = Rotations.of(0);
    public static final boolean kArmInverted                    = false;
    public static final double  kArmMaxVelocityRPM              = Degrees.of(90).per(Second).in(RPM);
    public static final double  kArmMaxAccelerationRPMperSecond = Degrees.of(180).per(Second).per(Second)
                                                                         .in(RPM.per(Second));
    public static final int     kArmStallCurrentLimitAmps       = 40;

    public static final double kArmkS = 0.2738; // volts (V)
    public static final double kArmkG = 0.039519; // volts (V)
    public static final double kArmKv = 0.10303; // volts per velocity (V/RPM)
    public static final double kArmKa = 0.085472; // volts per acceleration (V/(RPM/s))
  }

  public static class ElevatorConstants
  {

    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing    = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass       = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 10.25;
  }


}