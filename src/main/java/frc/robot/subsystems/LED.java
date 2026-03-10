/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  Spark ledStrip = new Spark(2);

  // LED Strip Lights:
  // Default Status: Solid Blue (0.83)
  // Coral in Intake: Blinking Yellow (-0.07) - Need to be green maybe - use
  // screwdriver with dial
  // Turn to Reef Mode: Solid Violet (0.91)
  // Not Within Tolerance: Blinking Red (-0.11)

//   private final Intake s_Intake;
//   private final Limelight s_Limelight;
//   private final BooleanSupplier isAligningToReef;

  // https://1166281274-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-ME3KPEhFI6-MDoP9nZD%2Fuploads%2FMOYJvZmWgxCVKJhcV5fn%2FREV-11-1105-LED-Patterns.pdf?alt=media&token=e8227890-6dd3-498d-834a-752fa43413fe

  // Change these variables to reflect the values
  public static final double BLUE_LIGHTS = 0.83;
  public static final double STROBE_GREEN = 0.35;
  public static final double VIOLET_LIGHTS = 0.91;
  public static final double STROBE_RED = -0.11;

  // Any subsystems being passed in should have data inside of it
  // For example, s_Intake.isCoralInIntake().
  public LED() {
    // setup each of the subsystems necessary
    // this.s_Intake = intake;
    // this.isAligningToReef = aligningToReef;
    // this.s_Limelight = limelight;
  }

  @Override
  public void periodic() {
    double lightPattern;

    // SmartDashboard.putBoolean("LED - isReefAprilTagValid", s_Limelight.getIsReefAprilTagValid());

    // You need to add additional if statements below for each LED color
    // if (isAligningToReef.getAsBoolean()) {
    //   if (s_Limelight.getIsReefAprilTagValid()) {
    //     SmartDashboard.putString("LED - color", "Solid Violet");
    //     lightPattern = VIOLET_LIGHTS;
    //   } else {
    //     SmartDashboard.putString("LED - color", "Strobe Red");
    //     lightPattern = STROBE_RED;
    //   }
    // } else if (s_Intake.isCoralInIntake()) {
    //   SmartDashboard.putString("LED - color", "Strobe Green");
    //   lightPattern = STROBE_GREEN;
    // } else {
    //   SmartDashboard.putString("LED - color", "Solid Blue");
    //   lightPattern = BLUE_LIGHTS;
    // }

    ledStrip.set(lightPattern);
  }

  public Spark getLEDController() {
    return ledStrip;
  }
}