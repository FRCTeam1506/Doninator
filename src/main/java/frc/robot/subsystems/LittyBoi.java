package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.RGBColor;
import frc.robot.subsystems.IndexerSubsystem.IndexerState;;

public class LittyBoi extends SubsystemBase {

    private static final int LED_COUNT = 30;

    private final RobotContainer robot;

    private CANdle candle = new CANdle(Constants.Candle.CAN_ID, "rio");

    public LittyBoi (RobotContainer container) {
        candle.configFactoryDefault();
        candle.configLEDType(LEDStripType.RGB);
        candle.configStatusLedState(true);
        candle.configLOSBehavior(false);
        candle.configBrightnessScalar(0.5);
        candle.configVBatOutput(VBatOutputMode.Modulated);

        this.robot = container;

        setTriggers();
    }

    public void setZeroBall () { setOddsToColor(RGBColor.RED); }
    public void setOneBall () { setOddsToColor(RGBColor.YELLOW); }
    public void setTwoBalls () { setOddsToColor(RGBColor.GREEN); }

    public void setSeeking () { setEvensToColor(RGBColor.GRAY); }
    public void setAligned () { setEvensToColor(RGBColor.PINK); }

    private void setOddsToColor (RGBColor color) {
        for (int i = 1; i <= LED_COUNT; i+=2) {
            candle.setLEDs(color.r, color.g, color.b, 0, i, 1);
        }
    }

    private void setEvensToColor (RGBColor color) {
        for (int i = 0; i <= LED_COUNT; i+=2) {
            candle.setLEDs(color.r, color.g, color.b, 0, i, 1);
        }
    }

    private void setTriggers () {
        new Button(() -> this.robot.indexer.getIndexerState() == IndexerState.EMPTY)
            .whenPressed(new InstantCommand(() -> this.setZeroBall(), this));

        new Button(() -> this.robot.indexer.getIndexerState() == IndexerState.HIGH || this.robot.indexer.getIndexerState() == IndexerState.LOW)
            .whenPressed(new InstantCommand(() -> this.setOneBall(), this));

        new Button(() -> this.robot.indexer.getIndexerState() == IndexerState.FULL)
            .whenPressed(new InstantCommand(() -> this.setTwoBalls(), this));


        new Button(() -> !this.robot.turret.limelightData.isAligned)
            .whenPressed(new InstantCommand(() -> this.setAligned(), this));
        
        new Button(() -> this.robot.turret.limelightData.isAligned)
            .whenPressed(new InstantCommand(() -> this.setAligned(), this));
    }
    
}
