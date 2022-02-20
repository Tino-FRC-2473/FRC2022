package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {
    enum DesiredMode {
        RED_3_BALL, 
        BLUE_3_BALL,
        RED_2_BALL,
        BLUE_2_BALL,
        RED_1_BALL,
        BLUE_1_BALL
    }

    private DesiredMode mDesiredMode = null;

    private SendableChooser<DesiredMode> modeChooser;

    public AutoSelector() {
        modeChooser = new SendableChooser<>();
        modeChooser.setDefaultOption("3 Ball Red", DesiredMode.RED_3_BALL);
        modeChooser.addOption("3 Ball Blue", DesiredMode.RED_3_BALL);
        modeChooser.addOption("2 Ball Red", DesiredMode.RED_2_BALL);
        modeChooser.addOption("2 Ball Blue", DesiredMode.BLUE_2_BALL);
        modeChooser.addOption("1 Ball Red", DesiredMode.RED_1_BALL);
        modeChooser.addOption("1 Ball Blue", DesiredMode.BLUE_1_BALL);
        SmartDashboard.putData("Auto mode", modeChooser);
    }

    public void updateModeChooser() {
        DesiredMode desiredMode = modeChooser.getSelected();
        if(mDesiredMode != desiredMode) {
            System.out.println("Auto selection changed: desiredMode->" + desiredMode.name());
        }
    }

    public void reset() {
        mDesiredMode = null;
    }

    public void outputToShuffleboard() {
        SmartDashboard.putString("Auto Mode selected", mDesiredMode.name());
    }
}
