package frc.robot.utilities;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class SparkFactory {

  /**
   * The default method to be used when creating a new CANSparkMax, which gives it basic settings
   * for ease of use.
   *
   * <p>MAKE SURE TO BURN FLASH!!!
   *
   * @param id
   * @param flipSparkMax
   * @return a new CAN object
   */
  public static CANSparkMax createCANSparkMax(int id, Boolean flipSparkMax) {

    // something to the effect of we experimented with burn flash and reset to
    // factory defaults, and it caused the drive motors to go bonkers
    CANSparkMax canToMake = new CANSparkMax(id, MotorType.kBrushless);
    canToMake.enableSoftLimit(SoftLimitDirection.kForward, false);
    canToMake.enableSoftLimit(SoftLimitDirection.kReverse, false);
    canToMake.setIdleMode(IdleMode.kBrake);
    canToMake.follow(ExternalFollower.kFollowerDisabled, 0);
    canToMake.setInverted(flipSparkMax);
    if (canToMake.isFollower()) {

      System.out.println(
          "CAN ID"
              + id
              + " is following somebody WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");
    }
    return canToMake;
  }

  /**
   * The default method to be used when creating a new CANSparkMax, which gives it basic settings
   * for ease of use.
   *
   * <p>MAKE SURE TO BURN FLASH!!!
   *
   * @param id
   * @return a new CAN object
   */
  public static CANSparkMax createCANSparkMax(int id) {

    return createCANSparkMax(id, false);
  }
}
