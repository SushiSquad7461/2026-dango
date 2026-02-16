// LED Control System for FRC Robot
// Displays: System Health, Endgame Alerts, Shooter Status, Hood Angle, Climb Progress

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private static LEDs instance;

  // ========== CONFIGURATION - SET THESE VALUES ==========
  // Strip 1 & 2: Shooter/Hood/Climb displays (mirrored on both sides)
  private static final int STRIP_1_PORT = 0; // TODO: Set PWM port for left strip
  private static final int STRIP_1_COUNT = 60; // TODO: Set LED count for left strip
  
  private static final int STRIP_2_PORT = 1; // TODO: Set PWM port for right strip
  private static final int STRIP_2_COUNT = 60; // TODO: Set LED count for right strip
  
  // Strip 3 & 4: System health, alerts, endgame (mirrored on both sides)
  private static final int STRIP_3_PORT = 2; // TODO: Set PWM port for left strip
  private static final int STRIP_3_COUNT = 30; // TODO: Set LED count for left strip
  
  private static final int STRIP_4_PORT = 3; // TODO: Set PWM port for right strip
  private static final int STRIP_4_COUNT = 30; // TODO: Set LED count for right strip
  // ======================================================

  // LED Hardware - Strips 1 & 2 (Shooter/Hood/Climb)
  private final AddressableLED strip1;
  private final AddressableLEDBuffer buffer1;
  private final AddressableLED strip2;
  private final AddressableLEDBuffer buffer2;
  
  // LED Hardware - Strips 3 & 4 (System/Alerts)
  private final AddressableLED strip3;
  private final AddressableLEDBuffer buffer3;
  private final AddressableLED strip4;
  private final AddressableLEDBuffer buffer4;

  // Robot State - Set these from your subsystems/commands
  public boolean visionDisconnected = false;
  public boolean canBusError = false;
  public boolean lowBattery = false;
  public boolean shooterAtSpeed = false;
  public double shooterSpeedPercent = 0.0; // 0.0 to 1.0
  public double hoodAngleDegrees = 0.0; // Your hood angle
  public double minHoodAngle = 0.0; // Min angle your hood can reach
  public double maxHoodAngle = 90.0; // Max angle your hood can reach
  public double climbProgress = 0.0; // 0.0 to 1.0 (0% to 100%)
  public boolean climbing = false;

  // Timing
  private double endgameStartTime = 135.0 - 30.0; // Last 30 seconds
  private double matchTime = 0.0;

  // Animation timing
  private double lastUpdateTime = 0.0;

  // LED Sections for Strips 1 & 2 (Shooter/Hood/Climb)
  private Section shooterSection1;
  private Section hoodSection1;
  private Section climbSection1;
  
  private Section shooterSection2;
  private Section hoodSection2;
  private Section climbSection2;
  
  // LED Sections for Strips 3 & 4 (System/Alerts)
  private Section systemHealthSection3;
  private Section alertSection3;
  
  private Section systemHealthSection4;
  private Section alertSection4;

  public static LEDs getInstance() {
    if (instance == null) {
      instance = new LEDs();
    }
    return instance;
  }

  private LEDs() {
    // Initialize Strip 1 & 2 (Shooter/Hood/Climb)
    strip1 = new AddressableLED(STRIP_1_PORT);
    buffer1 = new AddressableLEDBuffer(STRIP_1_COUNT);
    strip1.setLength(STRIP_1_COUNT);
    strip1.start();
    
    strip2 = new AddressableLED(STRIP_2_PORT);
    buffer2 = new AddressableLEDBuffer(STRIP_2_COUNT);
    strip2.setLength(STRIP_2_COUNT);
    strip2.start();
    
    // Initialize Strip 3 & 4 (System/Alerts)
    strip3 = new AddressableLED(STRIP_3_PORT);
    buffer3 = new AddressableLEDBuffer(STRIP_3_COUNT);
    strip3.setLength(STRIP_3_COUNT);
    strip3.start();
    
    strip4 = new AddressableLED(STRIP_4_PORT);
    buffer4 = new AddressableLEDBuffer(STRIP_4_COUNT);
    strip4.setLength(STRIP_4_COUNT);
    strip4.start();

    // Divide LED strips into sections
    calculateSections();

    System.out.println("LEDs initialized:");
    System.out.println("Strip 1 (Shooter/Hood/Climb): Port " + STRIP_1_PORT + ", " + STRIP_1_COUNT + " LEDs");
    System.out.println("  Shooter: " + shooterSection1);
    System.out.println("  Hood: " + hoodSection1);
    System.out.println("  Climb: " + climbSection1);
    System.out.println("Strip 2 (Shooter/Hood/Climb): Port " + STRIP_2_PORT + ", " + STRIP_2_COUNT + " LEDs");
    System.out.println("  Shooter: " + shooterSection2);
    System.out.println("  Hood: " + hoodSection2);
    System.out.println("  Climb: " + climbSection2);
    System.out.println("Strip 3 (System/Alerts): Port " + STRIP_3_PORT + ", " + STRIP_3_COUNT + " LEDs");
    System.out.println("  Health: " + systemHealthSection3);
    System.out.println("  Alerts: " + alertSection3);
    System.out.println("Strip 4 (System/Alerts): Port " + STRIP_4_PORT + ", " + STRIP_4_COUNT + " LEDs");
    System.out.println("  Health: " + systemHealthSection4);
    System.out.println("  Alerts: " + alertSection4);
  }

  private void calculateSections() {
    // Strips 1 & 2: Divide into thirds for Shooter/Hood/Climb
    // Each section uses full third of the strip length
    
    int shooter1Size = STRIP_1_COUNT / 3;
    int hood1Size = STRIP_1_COUNT / 3;
    int climb1Size = STRIP_1_COUNT - shooter1Size - hood1Size; // Use remaining
    
    shooterSection1 = new Section(0, shooter1Size);
    hoodSection1 = new Section(shooter1Size, shooter1Size + hood1Size);
    climbSection1 = new Section(shooter1Size + hood1Size, STRIP_1_COUNT);
    
    int shooter2Size = STRIP_2_COUNT / 3;
    int hood2Size = STRIP_2_COUNT / 3;
    int climb2Size = STRIP_2_COUNT - shooter2Size - hood2Size; // Use remaining
    
    shooterSection2 = new Section(0, shooter2Size);
    hoodSection2 = new Section(shooter2Size, shooter2Size + hood2Size);
    climbSection2 = new Section(shooter2Size + hood2Size, STRIP_2_COUNT);
    
    // Strips 3 & 4: Divide in half for System Health and Alerts
    // Each section uses half the strip length
    
    int health3Size = STRIP_3_COUNT / 2;
    int alert3Size = STRIP_3_COUNT - health3Size; // Use remaining
    
    systemHealthSection3 = new Section(0, health3Size);
    alertSection3 = new Section(health3Size, STRIP_3_COUNT);
    
    int health4Size = STRIP_4_COUNT / 2;
    int alert4Size = STRIP_4_COUNT - health4Size; // Use remaining
    
    systemHealthSection4 = new Section(0, health4Size);
    alertSection4 = new Section(health4Size, STRIP_4_COUNT);
  }

  @Override
  public void periodic() {
    lastUpdateTime = Timer.getFPGATimestamp();

    // Get match time for endgame alert
    matchTime = DriverStation.getMatchTime();

    // Clear all strips
    clearAll();

    // Priority 1: Critical alerts override everything
    if (checkCriticalAlerts()) {
      sendDataToAllStrips();
      return;
    }

    // ===== STRIPS 1 & 2: Shooter/Hood/Climb =====
    displayShooterStatus();
    displayHoodAngle();
    displayClimbProgress();

    // ===== STRIPS 3 & 4: System Health/Alerts =====
    displaySystemHealth();
    
    // Priority 2: Endgame alert (last 30 seconds)
    if (DriverStation.isTeleop() && matchTime > 0 && matchTime <= 30.0) {
      displayEndgameAlert();
    } else {
      displayAlertSection();
    }

    // Send data to all LED strips
    sendDataToAllStrips();

    // Record cycle time if using AdvantageKit
    // LoggedTracer.record("LEDs");
  }
  
  private void sendDataToAllStrips() {
    strip1.setData(buffer1);
    strip2.setData(buffer2);
    strip3.setData(buffer3);
    strip4.setData(buffer4);
  }

  // ==================== SYSTEM HEALTH (Strips 3 & 4) ====================
  private void displaySystemHealth() {
    // Green = all systems good
    // Yellow = minor issues
    // Red = critical issues

    Color healthColor = Color.kGreen;

    if (visionDisconnected) {
      healthColor = Color.kYellow;
    }
    if (canBusError) {
      healthColor = Color.kOrange;
    }
    if (lowBattery) {
      healthColor = Color.kRed;
    }

    // Solid color for good, blink for issues
    if (healthColor.equals(Color.kGreen)) {
      solid(buffer3, systemHealthSection3, healthColor);
      solid(buffer4, systemHealthSection4, healthColor);
    } else {
      blink(buffer3, systemHealthSection3, healthColor, Color.kBlack, 0.5);
      blink(buffer4, systemHealthSection4, healthColor, Color.kBlack, 0.5);
    }
  }

  // ==================== SHOOTER STATUS (Strips 1 & 2) ====================
  private void displayShooterStatus() {
    if (shooterAtSpeed) {
      // Solid green when at speed and ready
      solid(buffer1, shooterSection1, Color.kLime);
      solid(buffer2, shooterSection2, Color.kLime);
      // Optional: Add spinning effect
      // chase(buffer1, shooterSection1, Color.kLime, Color.kGreen, 0.1);
      // chase(buffer2, shooterSection2, Color.kLime, Color.kGreen, 0.1);
    } else if (shooterSpeedPercent > 0.0) {
      // Show progress as shooter spins up
      progressBar(buffer1, shooterSection1, Color.kYellow, Color.kBlack, shooterSpeedPercent);
      progressBar(buffer2, shooterSection2, Color.kYellow, Color.kBlack, shooterSpeedPercent);
    } else {
      // Off when not running
      solid(buffer1, shooterSection1, Color.kBlack);
      solid(buffer2, shooterSection2, Color.kBlack);
    }
  }

  // ==================== HOOD ANGLE (Strips 1 & 2) ====================
  private void displayHoodAngle() {
    // Map hood angle to color gradient
    // Low angle (flat) = Blue
    // Mid angle = Purple
    // High angle (steep) = Red

    double anglePercent =
        (hoodAngleDegrees - minHoodAngle) / (maxHoodAngle - minHoodAngle);
    anglePercent = Math.max(0.0, Math.min(1.0, anglePercent)); // Clamp 0-1

    Color lowColor = Color.kBlue; // Flat shot
    Color midColor = Color.kPurple; // Medium
    Color highColor = Color.kRed; // High arc

    Color hoodColor;
    if (anglePercent < 0.5) {
      // Interpolate between blue and purple
      hoodColor = lerpColor(lowColor, midColor, anglePercent * 2.0);
    } else {
      // Interpolate between purple and red
      hoodColor = lerpColor(midColor, highColor, (anglePercent - 0.5) * 2.0);
    }

    // Show as progress bar indicating angle
    progressBar(buffer1, hoodSection1, hoodColor, Color.kBlack, anglePercent);
    progressBar(buffer2, hoodSection2, hoodColor, Color.kBlack, anglePercent);
  }

  // ==================== CLIMB PROGRESS (Strips 1 & 2) ====================
  private void displayClimbProgress() {
    if (!climbing) {
      // Show dim white when not climbing
      solid(buffer1, climbSection1, new Color(0.1, 0.1, 0.1));
      solid(buffer2, climbSection2, new Color(0.1, 0.1, 0.1));
    } else {
      // Bright progress bar during climb
      progressBar(buffer1, climbSection1, Color.kGreen, Color.kBlack, climbProgress);
      progressBar(buffer2, climbSection2, Color.kGreen, Color.kBlack, climbProgress);

      // Flash when complete
      if (climbProgress >= 0.99) {
        blink(buffer1, climbSection1, Color.kLime, Color.kGreen, 0.2);
        blink(buffer2, climbSection2, Color.kLime, Color.kGreen, 0.2);
      }
    }
  }

  // ==================== ALERTS (Strips 3 & 4) ====================
  private void displayAlertSection() {
    // Use this section for miscellaneous alerts
    solid(buffer3, alertSection3, Color.kBlack);
    solid(buffer4, alertSection4, Color.kBlack);
  }

  private void displayEndgameAlert() {
    // Flash red/yellow during last 30 seconds
    double flashSpeed = 0.5;
    if (matchTime <= 10.0) {
      flashSpeed = 0.2; // Faster in last 10 seconds
    }
    blink(buffer3, alertSection3, Color.kRed, Color.kYellow, flashSpeed);
    blink(buffer4, alertSection4, Color.kRed, Color.kYellow, flashSpeed);
  }

  private boolean checkCriticalAlerts() {
    // Check for emergency stop or critical failures
    if (DriverStation.isEStopped()) {
      // Fill all strips with red
      for (int i = 0; i < STRIP_1_COUNT; i++) {
        buffer1.setLED(i, Color.kRed);
      }
      for (int i = 0; i < STRIP_2_COUNT; i++) {
        buffer2.setLED(i, Color.kRed);
      }
      for (int i = 0; i < STRIP_3_COUNT; i++) {
        buffer3.setLED(i, Color.kRed);
      }
      for (int i = 0; i < STRIP_4_COUNT; i++) {
        buffer4.setLED(i, Color.kRed);
      }
      return true;
    }
    return false;
  }

  // ==================== LED PATTERN FUNCTIONS ====================

  private void clearAll() {
    for (int i = 0; i < STRIP_1_COUNT; i++) {
      buffer1.setLED(i, Color.kBlack);
    }
    for (int i = 0; i < STRIP_2_COUNT; i++) {
      buffer2.setLED(i, Color.kBlack);
    }
    for (int i = 0; i < STRIP_3_COUNT; i++) {
      buffer3.setLED(i, Color.kBlack);
    }
    for (int i = 0; i < STRIP_4_COUNT; i++) {
      buffer4.setLED(i, Color.kBlack);
    }
  }

  private void solid(AddressableLEDBuffer buffer, Section section, Color color) {
    for (int i = section.start; i < section.end; i++) {
      buffer.setLED(i, color);
    }
  }

  private void blink(AddressableLEDBuffer buffer, Section section, Color c1, Color c2, double duration) {
    boolean useFirst = ((lastUpdateTime % duration) / duration) < 0.5;
    solid(buffer, section, useFirst ? c1 : c2);
  }

  private void progressBar(AddressableLEDBuffer buffer, Section section, Color fillColor, Color bgColor, double percent) {
    percent = Math.max(0.0, Math.min(1.0, percent));
    int fillCount = (int) ((section.end - section.start) * percent);

    for (int i = section.start; i < section.end; i++) {
      if (i < section.start + fillCount) {
        buffer.setLED(i, fillColor);
      } else {
        buffer.setLED(i, bgColor);
      }
    }
  }

  private void chase(AddressableLEDBuffer buffer, Section section, Color c1, Color c2, double speed) {
    int offset = (int) ((lastUpdateTime / speed) % (section.end - section.start));
    for (int i = section.start; i < section.end; i++) {
      int pos = (i - section.start + offset) % (section.end - section.start);
      buffer.setLED(i, pos % 2 == 0 ? c1 : c2);
    }
  }

  private Color lerpColor(Color c1, Color c2, double t) {
    t = Math.max(0.0, Math.min(1.0, t));
    return new Color(
        c1.red + (c2.red - c1.red) * t,
        c1.green + (c2.green - c1.green) * t,
        c1.blue + (c2.blue - c1.blue) * t);
  }

  // ==================== PUBLIC SETTERS ====================
  // Call these from your other subsystems

  public void setShooterStatus(boolean atSpeed, double speedPercent) {
    this.shooterAtSpeed = atSpeed;
    this.shooterSpeedPercent = speedPercent;
  }

  public void setHoodAngle(double degrees, double min, double max) {
    this.hoodAngleDegrees = degrees;
    this.minHoodAngle = min;
    this.maxHoodAngle = max;
  }

  public void setClimbProgress(double progress, boolean isClimbing) {
    this.climbProgress = progress;
    this.climbing = isClimbing;
  }

  public void setSystemHealth(boolean visionOk, boolean canBusOk, boolean batteryOk) {
    this.visionDisconnected = !visionOk;
    this.canBusError = !canBusOk;
    this.lowBattery = !batteryOk;
  }

  // Helper class for LED sections
  private record Section(int start, int end) {
    @Override
    public String toString() {
      return "[" + start + "-" + end + "] (" + (end - start) + " LEDs)";
    }
  }
}