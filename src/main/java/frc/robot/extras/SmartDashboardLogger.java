package frc.robot.extras;

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
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoString(final String key, final String value) {
        return putString(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnString(final String key, final String value) {
        return putString(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorString(final String key, final String value) {
        return putString(SmartDashboardLogLevel.ERROR, key, value);
    }

    /**
     * Put a string in the table if logLevel is null or if logLevel >= {@code systemLogLevel}.
     *
     * @param logLevel log level required to put this string
     * @param key      the key to be assigned to
     * @param value    the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    private static boolean putString(final SmartDashboardLogLevel logLevel, final String key, final String value) {
        if (isNull(logLevel) || logLevel.getLevel() >= systemLogLevel.getLevel()) {
            return SmartDashboard.putString(key, value);
        }
        return true;
    }

    /**
     * Put a boolean in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugBoolean(final String key, final boolean value) {
        return putBoolean(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoBoolean(final String key, final boolean value) {
        return putBoolean(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnBoolean(final String key, final boolean value) {
        return putBoolean(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorBoolean(final String key, final boolean value) {
        return putBoolean(SmartDashboardLogLevel.ERROR, key, value);
    }
    /**
     * Put a string in the table if logLevel is null or if logLevel >= {@code systemLogLevel}.
     *
     * @param logLevel log level required to put this string
     * @param key      the key to be assigned to
     * @param value    the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    private static boolean putBoolean(final SmartDashboardLogLevel logLevel, final String key, final boolean value) {
        if (isNull(logLevel) || logLevel.getLevel() >= systemLogLevel.getLevel()) {
            return SmartDashboard.putBoolean(key, value);
        }
        return true;
    }
    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugNumber(final String key, final double value) {
        return putNumber(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoNumber(final String key, final double value) {
        return putNumber(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnNumber(final String key, final double value) {
        return putNumber(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorNumber(final String key, final double value) {
        return putNumber(SmartDashboardLogLevel.ERROR, key, value);
    }

    /**
     * Put a number in the table if logLevel is null or if logLevel >= {@code systemLogLevel}.
     *
     * @param logLevel log level required to put this string
     * @param key      the key to be assigned to
     * @param value    the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    private static boolean putNumber(final SmartDashboardLogLevel logLevel, final String key, final double value) {
        if (isNull(logLevel) || logLevel.getLevel() >= systemLogLevel.getLevel()) {
            return SmartDashboard.putNumber(key, value);
        }

        return true;
    }

}
