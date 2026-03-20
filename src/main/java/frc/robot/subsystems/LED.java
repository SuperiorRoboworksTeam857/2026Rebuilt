/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  Spark ledStrip = new Spark(0);

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

  public static final double GREEN = 0.77;
  public static final double YELLOW = 0.69;
  public static final double RED = 0.61;
  public static final double WHITE = 0.93;

  public LED() {}

  @Override
  public void periodic() {
    double lightPattern = 0.01;

    String hubActiveColor = "#FFFFFF";
    HUB_STATE state = isHubActive();
    switch (state) {
      case ACTIVE:
        hubActiveColor = "#4CAF50";
        lightPattern = GREEN;
        break;
      case INACTIVE:
        hubActiveColor = "#F44336";
        lightPattern = RED;
        break;
      case ABOUT_TO_BE_ACTIVE:
        hubActiveColor = "#FFFFFF";
        lightPattern = WHITE;
        break;
      case ABOUT_TO_BE_INACTIVE:
        hubActiveColor = "#CCCC00";
        lightPattern = YELLOW;
        break;
    }

    SmartDashboard.putString("Hub Active", hubActiveColor);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    ledStrip.set(lightPattern); 
  }

  public Spark getLEDController() {
    return ledStrip;
  }

  public enum HUB_STATE {
    INACTIVE,
    ABOUT_TO_BE_ACTIVE,
    ACTIVE,
    ABOUT_TO_BE_INACTIVE
  }

  public HUB_STATE isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return HUB_STATE.INACTIVE;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return HUB_STATE.ACTIVE;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return HUB_STATE.INACTIVE;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as it's likely early in teleop.
    if (gameData.isEmpty()) {
      return HUB_STATE.ACTIVE;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return HUB_STATE.ACTIVE;
      }
    }

    // Shift 1 is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    final double EndOfTransition = 130;
    final double EndOfShift1 = 105;
    final double EndOfShift2 = 80;
    final double EndOfShift3 = 55;
    final double EndOfShift4 = 30;
    final double Margin = 3;

    if (shift1Active) {
      if (matchTime > EndOfTransition) {
        // Transition shift, hub is active.
        return HUB_STATE.ACTIVE;

      } else if (matchTime > EndOfShift1) {
        // Shift 1
        return HUB_STATE.ACTIVE;

      } else if (matchTime > EndOfShift1 - Margin) {
        // Shift 1 -> 2
        return HUB_STATE.ABOUT_TO_BE_INACTIVE;
      } else if (matchTime > EndOfShift2 + Margin) {
        // Shift 2
        return HUB_STATE.INACTIVE;
      } else if (matchTime > EndOfShift2) {
        // Shift 2 -> 3
        return HUB_STATE.ABOUT_TO_BE_ACTIVE;

      } else if (matchTime > EndOfShift3) {
        // Shift 3
        return HUB_STATE.ACTIVE;

      } else if (matchTime > EndOfShift3 - Margin) {
        // Shift 3 -> 4
        return HUB_STATE.ABOUT_TO_BE_INACTIVE;
      } else if (matchTime > EndOfShift4 + Margin) {
        // Shift 4
        return HUB_STATE.INACTIVE;
      } else if (matchTime > EndOfShift4) {
        // Shift 4 -> end game
        return HUB_STATE.ABOUT_TO_BE_ACTIVE;

      } else {
        // End game, hub always active.
        return HUB_STATE.ACTIVE;
      }
    }
    else {
      if (matchTime > EndOfTransition) {
        // Transition shift, hub is active.
        return HUB_STATE.ACTIVE;
        
      } else if (matchTime > EndOfTransition - Margin) {
        // Transition Shift -> Shift 1
        return HUB_STATE.ABOUT_TO_BE_INACTIVE;
      } else if (matchTime > EndOfShift1 + Margin) {
        // Shift 1
        return HUB_STATE.INACTIVE;
      } else if (matchTime > EndOfShift1) {
        // Shift 1 -> 2
        return HUB_STATE.ABOUT_TO_BE_ACTIVE;

      } else if (matchTime > EndOfShift2) {
        // Shift 2
        return HUB_STATE.ACTIVE;

      } else if (matchTime > EndOfShift2 - Margin) {
        // Shift 2 -> 3
        return HUB_STATE.ABOUT_TO_BE_INACTIVE;
      } else if (matchTime > EndOfShift3 + Margin) {
        // Shift 3
        return HUB_STATE.INACTIVE;
      } else if (matchTime > EndOfShift3) {
        // Shift 3 -> 4
        return HUB_STATE.ABOUT_TO_BE_ACTIVE;

      } else if (matchTime > EndOfShift4) {
        // Shift 4
        return HUB_STATE.ACTIVE;

      } else {
        // End game, hub always active.
        return HUB_STATE.ACTIVE;
      }
    }
  }
}