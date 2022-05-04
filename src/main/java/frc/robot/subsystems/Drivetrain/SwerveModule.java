// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TalonFXConstants;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

public class SwerveModule extends SubsystemBase {
    /*
     * Motion Magic Should be used for the turning PID with a calculated feed forward gain.
     * https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
     * 
     */
    
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    TalonFXConfiguration config;


    /** Creates a new SwerveModule. */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {   
    
    
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new CANCoder(absoluteEncoderId);
      
      driveMotor = new TalonFX(driveMotorId, "rio");
      turningMotor = new TalonFX(turningMotorId, "rio");

      driveMotor.setInverted(driveMotorReversed);
      turningMotor.setInverted(turningMotorReversed);

      /* Set Motion Magic gains in slot0 - see documentation */
      driveMotor.selectProfileSlot(ModuleConstants.kDriveSlotIdx, ModuleConstants.kPIDLoopIdx);
      driveMotor.config_kF(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kF, ModuleConstants.kTimeoutMs);
      driveMotor.config_kP(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kP, ModuleConstants.kTimeoutMs);
      driveMotor.config_kI(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kI, ModuleConstants.kTimeoutMs);
      driveMotor.config_kD(ModuleConstants.kDriveSlotIdx, ModuleConstants.kDriveGains.kD, ModuleConstants.kTimeoutMs);

      /* Set Motion Magic gains in slot1 - see documentation */
      driveMotor.selectProfileSlot(ModuleConstants.kTurnSlotIdx, ModuleConstants.kPIDLoopIdx);
      driveMotor.config_kF(ModuleConstants.kTurnSlotIdx, ModuleConstants.kTurnGains.kF, ModuleConstants.kTimeoutMs);
      driveMotor.config_kP(ModuleConstants.kTurnSlotIdx, ModuleConstants.kTurnGains.kP, ModuleConstants.kTimeoutMs);
      driveMotor.config_kI(ModuleConstants.kTurnSlotIdx, ModuleConstants.kTurnGains.kI, ModuleConstants.kTimeoutMs);
      driveMotor.config_kD(ModuleConstants.kTurnSlotIdx, ModuleConstants.kTurnGains.kD, ModuleConstants.kTimeoutMs);
      resetEncoders();
    }

    public double getDrivePosition() {
      /* Position in meters */
        return (driveMotor.getSelectedSensorPosition() / ModuleConstants.kUnitsPerRevolution) * ModuleConstants.kDriveEncoderRot2Meter; // Fix to constants
    }

    public double getTurningPosition() {
      /* Position in radians */
        return (turningMotor.getSelectedSensorPosition() / ModuleConstants.kUnitsPerRevolution) * 2 * Math.PI; // FIX to constants
    }

    public double getDriveVelocity() {
      /* Velocity in meters per second */
        return (driveMotor.getSelectedSensorVelocity() / 2080) * 0.1;
    }

    public double getTurningVelocity() {
      /* Position in radians per second*/
        return ((turningMotor.getSelectedSensorVelocity() / 2080) * 0.1) * 2 * Math.PI;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
      driveMotor.setSelectedSensorPosition(0);
      turningMotor.setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition() / 360);
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
      if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
      }
      state = SwerveModuleState.optimize(state, getState().angle);
      /**
       * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java 
       * 
       * Set Motion Magic gains in slot0 - see documentation 
       */
 
      driveMotor.set(ControlMode.MotionMagic, state.speedMetersPerSecond); // No idea if this will work lol
      turningMotor.set(ControlMode.MotionMagic, state.angle.getRadians()); // Must have continuous motion


      SmartDashboard.putString("Swerve[" + absoluteEncoder.getAbsolutePosition() + "] state", state.toString());
    }

    public void stop() {
      driveMotor.set(ControlMode.PercentOutput, 0);
      turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
