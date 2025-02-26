// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterUtils;

public class Leds {

  private CANdle candle;

  /** Creates a new Leds. */
  public Leds(int deviceId) {
    candle = new CANdle(deviceId, "*");
    CANdleConfiguration config = new CANdleConfiguration();
    candle.configAllSettings(config);
  }

  public void update() {
    // boolean a = true;
    // if(a) {
    //   candle.clearAnimation(0);
    //   return;
    // }

    // This method will be called once per scheduler run
    if(DriverStation.isDisabled()) {
      if(DriverStation.isFMSAttached()) {
        candle.animate(LedState.WHITE_TWINKLE.animation);
      } else {
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
          candle.animate(LedState.BLUE_TWINKLE.animation);
        } else {
          candle.animate(LedState.RED_TWINKLE.animation);
        }
      }

      return;
    }

    Animation anim = LedState.PURPLE_FADE.animation;
    SubsystemManager sm = SubsystemManager.getInstance();
    if(sm.getIntake().getState() == IntakeState.DEPLOYING || sm.getIntake().getState() == IntakeState.DEPLOYED) {
      anim = LedState.PURPLE_FADE.animation;
    } else if(sm.getIntake().getState() == IntakeState.TRANSFER || sm.getIntake().getState() == IntakeState.FEED) {
      anim = LedState.GREEN_BLINKY_BLINK.animation;
    } else if(ShooterUtils.isAligning()) {
      anim = LedState.YELLOW_LARSON.animation;
    } else if(ShooterUtils.isShooting()) {
      anim = LedState.RGB.animation;
    } else if(sm.getShooter().getState() == ShooterState.HOLDING) {
      anim = LedState.GREEN_SOLID.animation;
    }

    candle.animate(anim);
  }

  public enum LedState {
    WHITE_TWINKLE(new SingleFadeAnimation(255, 255, 255, 255, 0.7, 66)),
    BLUE_TWINKLE(new SingleFadeAnimation(0, 0, 255, 0, 0.7, 66)),
    RED_TWINKLE(new SingleFadeAnimation(255, 0, 0, 0, 0.7, 66)),
    PURPLE_FADE(new SingleFadeAnimation(255, 0, 255, 0, 0.4, 66, 0)),
    ORANGE_FLOW(new ColorFlowAnimation(255, 165, 0, 0, 0.4, 66, Direction.Forward)),
    GREEN_SOLID(new SingleFadeAnimation(0, 255, 0, 0, 0, 66, 0)),
    GREEN_BLINKY_BLINK(new StrobeAnimation(0, 255, 0, 0, 0.5, 66, 0)),
    YELLOW_LARSON(new LarsonAnimation(255, 255, 0, 0, 1, 66, BounceMode.Front, 8)),
    RGB(new RgbFadeAnimation());

    public Animation animation;
    private LedState(Animation animation) {
      this.animation = animation;
    }

  }
}
