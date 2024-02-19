// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightstripConstants;
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.Range;
import frc.robot.lightstrip.TempLedState;
import java.util.ArrayList;
import java.util.List;

public class Lightstrip extends SubsystemBase {
  CANdle candle = new CANdle(LightstripConstants.candlePort);
  RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, LightstripConstants.ledCount + 8);

  private LedState defaultColor = LightstripConstants.defaultState;
  private Timer timer = new Timer();
  private List<LedState> currentColor = new ArrayList<LedState>();
  private List<Range> currentRange = new ArrayList<Range>();
  private List<TempLedState> tempColor = new ArrayList<TempLedState>();
  private List<Range> tempRange = new ArrayList<Range>();
  private List<Double> tempStart = new ArrayList<Double>();

  private boolean isDefault = false;

  /** Creates a new Lightstrip. */
  public Lightstrip() {
    timer.reset();
    timer.start();

    isDefault = false;

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.75;
    candle.configAllSettings(config);

    setColor(new LedState(255, 0, 0, "Solid"), LightstripConstants.Ranges.intake);
    setColor(new LedState(0, 255, 0, "Solid"), LightstripConstants.Ranges.drivetrain);
    setColor(new LedState(0, 0, 255, "Solid"), LightstripConstants.Ranges.superstructure);
    setColor(new LedState(0, 255, 255, "Solid"), LightstripConstants.Ranges.shooter);
  }

  @Override
  public void periodic() {
    if (isDefault) {
      System.out.println("Default Lightstrip: " + isDefault);
      return;
    }

    candle.clearAnimation(0);

    update(defaultColor, timer, LightstripConstants.Ranges.full);

    for (int i = 0; i < currentColor.size(); i++) {
      update(currentColor.get(i), timer, currentRange.get(i));
    }

    List<TempLedState> states = new ArrayList<TempLedState>();
    List<Range> ranges = new ArrayList<Range>();
    List<Double> starts = new ArrayList<Double>();

    for (int i = 0; i < tempColor.size(); i++) {
      update(tempColor.get(i), timer, tempRange.get(i));

      if (tempStart.get(i) <= timer.get() + tempColor.get(i).getSeconds()) {
        states.add(tempColor.get(i));
        ranges.add(tempRange.get(i));
        starts.add(tempStart.get(i));
      }
    }

    tempColor = states;
    tempRange = ranges;
    tempStart = starts;
  }

  public void setDefault(boolean defaultState) {
    isDefault = defaultState;

    if (isDefault) {
      candle.animate(rainbowAnim);
    }
  }

  private void update(LedState state, Timer timer, Range range) {
    SmartDashboard.putNumber("Timer", timer.get() % 1 - 0.50 * state.getRed());
    if (state.getEffect() == "Solid") {
      candle.setLEDs(
          state.getRed(),
          state.getGreen(),
          state.getBlue(),
          0,
          range.getStart(),
          range.getEnd() - range.getStart() + 1);
    } else if (state.getEffect() == "Blink") {
      if ((timer.get() % 2) < 1) {
        candle.setLEDs(
            state.getRed(),
            state.getGreen(),
            state.getBlue(),
            0,
            range.getStart(),
            range.getEnd() - range.getStart() + 1);
      } else if ((timer.get() % 2) >= 1) {
        candle.setLEDs(0, 0, 0);
      }
    } else if (state.getEffect() == "Fast Blink") {
      if ((timer.get() % 0.5) < 0.25) {
        candle.setLEDs(
            state.getRed(),
            state.getGreen(),
            state.getBlue(),
            0,
            range.getStart(),
            range.getEnd() - range.getStart() + 1);
      } else if ((timer.get() % 0.5) >= 0.25) {
        candle.setLEDs(0, 0, 0);
      }
    } else if (state.getEffect() == "Fade") {
      candle.setLEDs(
          (int) (state.getRed() / ((timer.get() % 0.5) * 2)),
          (int) (state.getGreen() / ((timer.get() % 0.5) * 2)),
          (int) (state.getBlue() / ((timer.get() % 0.5) * 2)),
          0,
          range.getStart(),
          range.getEnd() - range.getStart() + 1);
    } else if (state.getEffect() == "Signal") {
      int chain = 0;

      if ((timer.get() % 1)
          < 0.125) { // i know there is prob a better way to do this, but im too lazy to
        chain = 0;
      } else if ((timer.get() % 1) < 0.25) {
        chain = 1;
      } else if ((timer.get() % 1) < 0.375) {
        chain = 2;
      } else if ((timer.get() % 1) < 0.5) {
        chain = 3;
      } else if ((timer.get() % 1) < 0.625) {
        chain = 4;
      } else if ((timer.get() % 1) < 0.75) {
        chain = 5;
      } else if ((timer.get() % 1) < 0.875) {
        chain = 6;
      } else if ((timer.get() % 1) < 1) {
        chain = 7;
      }

      for (int i = 0; i < range.getStart() - range.getEnd() + 1; i++) {
        int ratio = 6 - chain;

        if (ratio < 0) {
          ratio = 0;
        }

        candle.setLEDs(
            (int) (state.getRed() * chain / 6),
            (int) (state.getGreen() * chain / 6),
            (int) (state.getBlue() * chain / 6),
            0,
            range.getStart() + i,
            1);

        chain++;

        if (chain == 8) {
          chain = 0;
        }
      }
    } else if (state.getEffect() == "Wave") {
      int chain = 0;

      if ((timer.get() % 1)
          < 0.125) { // i know there is prob a better way to do this, but im too lazy to
        chain = 0;
      } else if ((timer.get() % 1) < 0.25) {
        chain = 1;
      } else if ((timer.get() % 1) < 0.375) {
        chain = 2;
      } else if ((timer.get() % 1) < 0.5) {
        chain = 3;
      } else if ((timer.get() % 1) < 0.625) {
        chain = 4;
      } else if ((timer.get() % 1) < 0.75) {
        chain = 5;
      } else if ((timer.get() % 1) < 0.875) {
        chain = 6;
      } else if ((timer.get() % 1) < 1) {
        chain = 7;
      }

      for (int i = 0; i < range.getStart() - range.getEnd() + 1; i++) {
        int ratio = 4 - chain;

        if (ratio < 0) {
          ratio = chain - 5;
        }

        candle.setLEDs(
            (int) (state.getRed() * chain / 6),
            (int) (state.getGreen() * chain / 6),
            (int) (state.getBlue() * chain / 6),
            0,
            range.getStart() + i,
            1);

        chain++;

        if (chain == 8) {
          chain = 0;
        }
      }
    }
  }

  public void toggleOnColor(LedState state, Range range) {
    setColor(state, range);
  }

  public void toggleOffColor(LedState state, Range range) {
    List<LedState> states = new ArrayList<LedState>();
    List<Range> ranges = new ArrayList<Range>();

    for (int i = 0; i < currentColor.size(); i++) {
      if (!(currentColor.get(i).equals(state) && currentRange.get(i).equals(range))) {
        states.add(state);
        ranges.add(range);
      }
    }

    currentColor = states;
    currentRange = ranges;
  }

  public void setColor(LedState state, Range range) {
    toggleOffColor(state, range);

    currentColor.add(state);
    currentRange.add(range);
  }

  public void tempColor(TempLedState state, Range range) {
    tempColor.add(state);
    tempRange.add(range);
    tempStart.add(Double.valueOf(timer.get()));
  }
}
