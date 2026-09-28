package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.StringReader;
import java.io.StringWriter;
import java.util.Properties;

/**
 * Parameters shared by the tuner and operate opmodes.
 * The tuner saves them to the Robot Controller's settings folder; the operate opmode loads them.
 *
 * Units: velocity in flywheel RPM, voltage in volts, time in ms.
 */
public class FlywheelParams {
    private static final String FILE_NAME = "FlywheelParams.properties";

    // ---- Hardware ----
    // goBilda yellow jacket 5203-2402-0019
    // with conversion to 1:1 gear ratio
    // 6,000 RPM
    /** Encoder ticks per FLYWHEEL revolution (include any gear ratio between motor and wheel). */
    public double ticksPerRev = 28.0;

    // ---- Feedforward / feedback (volts) ----
    /** Static friction voltage (intercept of volts vs. RPM). */
    public double kS = 0.40;
    /** Back-EMF constant: volts per RPM at steady state (slope of volts vs. RPM). */
    public double kV = 0.0019;
    /** Proportional gain: volts per RPM of error. */
    public double kP = 0.004;

    // ---- The two key tuned parameters ----
    /**
     * Fraction of battery voltage held in reserve so the controller can still push after a ball
     * loads the wheel. Max sustainable target = (Vbat * (1 - headroom) - kS) / kV.
     */
    public double backEmfHeadroom = 0.15;
    /** Flywheel is "ready" when |target - rpm| <= toleranceRpm ... */
    public double toleranceRpm = 75.0;
    /** ... continuously for this long. */
    public double readyDwellMs = 30.0;

    // ---- Misc ----
    /** Low-pass coefficient on measured velocity (1 = no filtering). */
    public double velFilterAlpha = 0.40;
    public double targetRpm = 2680.0;

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static FlywheelParams load() {
        FlywheelParams p = new FlywheelParams();
        File f = file();
        if (!f.exists()) return p;
        try {
            Properties props = new Properties();
            props.load(new StringReader(ReadWriteFile.readFile(f)));
            p.ticksPerRev = get(props, "ticksPerRev", p.ticksPerRev);
            p.kS = get(props, "kS", p.kS);
            p.kV = get(props, "kV", p.kV);
            p.kP = get(props, "kP", p.kP);
            p.backEmfHeadroom = get(props, "backEmfHeadroom", p.backEmfHeadroom);
            p.toleranceRpm = get(props, "toleranceRpm", p.toleranceRpm);
            p.readyDwellMs = get(props, "readyDwellMs", p.readyDwellMs);
            p.velFilterAlpha = get(props, "velFilterAlpha", p.velFilterAlpha);
            p.targetRpm = get(props, "targetRpm", p.targetRpm);
        } catch (Exception e) {
            throw new RuntimeException("Corrupt file: fall back to defaults.", e);
        }
        return p;
    }

    public void save() {
        try {
            Properties props = new Properties();
            props.setProperty("ticksPerRev", Double.toString(ticksPerRev));
            props.setProperty("kS", Double.toString(kS));
            props.setProperty("kV", Double.toString(kV));
            props.setProperty("kP", Double.toString(kP));
            props.setProperty("backEmfHeadroom", Double.toString(backEmfHeadroom));
            props.setProperty("toleranceRpm", Double.toString(toleranceRpm));
            props.setProperty("readyDwellMs", Double.toString(readyDwellMs));
            props.setProperty("velFilterAlpha", Double.toString(velFilterAlpha));
            props.setProperty("targetRpm", Double.toString(targetRpm));
            StringWriter sw = new StringWriter();
            props.store(sw, "Flywheel tuning");
            ReadWriteFile.writeFile(file(), sw.toString());
        } catch (Exception e) {
            throw new RuntimeException("Failed to save flywheel params", e);
        }
    }

    private static double get(Properties props, String key, double def) {
        String s = props.getProperty(key);
        if (s == null) return def;
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return def;
        }
    }
}
