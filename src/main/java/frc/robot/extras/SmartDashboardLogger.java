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
    private static final SmartDashboardLogLevel systemLogLevel = SmartDashboardLogLevel.INFO;

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugString(final String key, final String value) {
        return put(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoString(final String key, final String value) {
        return put(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnString(final String key, final String value) {
        return put(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a string in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorString(final String key, final String value) {
        return put(SmartDashboardLogLevel.ERROR, key, value);
    }

    /**
     * Put a boolean in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugBoolean(final String key, final Boolean value) {
        return put(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a boolean in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoBoolean(final String key, final Boolean value) {
        return put(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a boolean in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnBoolean(final String key, final Boolean value) {
        return put(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a boolean in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorBoolean(final String key, final Boolean value) {
        return put(SmartDashboardLogLevel.ERROR, key, value);
    }

    /**
     * Put a double[] in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugNumberArray(final String key, final double[] value) {
        return put(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a double[] in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoNumberArray(final String key, final double[] value) {
        return put(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a double[] in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnNumberArray(final String key, final double[] value) {
        return put(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a double[] in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorNumberArray(final String key, final double[] value) {
        return put(SmartDashboardLogLevel.ERROR, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#DEBUG}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean debugNumber(final String key, final double value) {
        return put(SmartDashboardLogLevel.DEBUG, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#INFO}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean infoNumber(final String key, final double value) {
        return put(SmartDashboardLogLevel.INFO, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#WARN}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean warnNumber(final String key, final double value) {
        return put(SmartDashboardLogLevel.WARN, key, value);
    }

    /**
     * Put a number in the table at logLevel {@link SmartDashboardLogLevel#ERROR}
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    public static boolean errorNumber(final String key, final double value) {
        return put(SmartDashboardLogLevel.ERROR, key, value);
    }

    /**
     * Put an object in the table if logLevel is null or if logLevel >= {@code systemLogLevel}.
     *
     * @param logLevel log level required to put this object
     * @param key      the key to be assigned to
     * @param value    the value that will be assigned
     * @return False if the table key already exists with a different type
     */
    private static boolean put(final SmartDashboardLogLevel logLevel, final String key, final Object value) {
        if (isNull(logLevel) || logLevel.getLevel() >= systemLogLevel.getLevel()) {
            if (value instanceof String) {
                return SmartDashboard.putString(key, (String) value);
            } else if (value instanceof Double) {
                return SmartDashboard.putNumber(key, (Double) value);
            } else if (value instanceof Boolean) {
                return SmartDashboard.putBoolean(key, (Boolean) value);
            } else if (value instanceof double[]) {
                return SmartDashboard.putNumberArray(key, (double[]) value);
            }
        }

        return true;
    }

}
