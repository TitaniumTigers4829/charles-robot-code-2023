package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static java.util.Objects.isNull;

public final class SmartDashboardLogger {

    // prevent instantiation
    private SmartDashboardLogger() {
    }

    /*
     * TODO - maybe read from config
     */
    private static final SmartDashboardLogLevel systemLogLevel = SmartDashboardLogLevel.DEBUG;

    /**
     * Put a string in the table regardless of {@code systemLogLevel}.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putString(final String key, final String value) {
        return putString(null, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugString(final String key, final String value) {
        return putString(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a string in the table if logLevel is null or if logLevel >= {@code systemLogLevel}.
     *
     * @param logLevel log level required to put this string
     * @param key      the key to be assigned to
     * @param value    the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putString(final SmartDashboardLogLevel logLevel, final String key, final String value) {
        if (isNull(logLevel) || logLevel.getLevel() >= systemLogLevel.getLevel()) {
            return SmartDashboard.putString(key, value);
        }

        return true;
    }

    /**
     * Put a number in the table regardless of {@code systemLogLevel}.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugNumber(final String key, final double value) {
        return putNumber(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putNumber(final String key, final double value) {
        return putNumber(null, key, value);
    }

    /**
     * Put a number in the table if logLevel is null or if logLevel >= {@code systemLogLevel}.
     *
     * @param logLevel log level required to put this string
     * @param key      the key to be assigned to
     * @param value    the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean putNumber(final SmartDashboardLogLevel logLevel, final String key, final double value) {
        if (isNull(logLevel) || logLevel.getLevel() >= systemLogLevel.getLevel()) {
            return SmartDashboard.putNumber(key, value);
        }

        return true;
    }

}
