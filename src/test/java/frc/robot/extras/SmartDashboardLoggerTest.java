package frc.robot.extras;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SmartDashboardLoggerTest {

    private static final Boolean TEST_BOOLEAN = Boolean.FALSE;

    private static final double[] TEST_NUMBER_ARRAY = {0.1, 0.2, 0.4};

    private static final double TEST_NUMBER = 0.125;

    private static final String TEST_STRING = "test value";

    private static int keyIndex = 0;

    @Test
    void debugBoolean() {
        assertTrue(SmartDashboardLogger.debugBoolean(nextKey(), TEST_BOOLEAN));
    }

    @Test
    void infoBoolean() {
        assertTrue(SmartDashboardLogger.infoBoolean(nextKey(), TEST_BOOLEAN));
    }

    @Test
    void warnBoolean() {
        assertTrue(SmartDashboardLogger.warnBoolean(nextKey(), TEST_BOOLEAN));
    }

    @Test
    void errorBoolean() {
        assertTrue(SmartDashboardLogger.errorBoolean(nextKey(), TEST_BOOLEAN));
    }

    @Test
    void errorBoolean_originally_string() {
        final String key = nextKey();
        assertTrue(SmartDashboardLogger.errorString(key, TEST_STRING));
        assertFalse(SmartDashboardLogger.errorBoolean(key, TEST_BOOLEAN));
    }

    @Test
    void debugNumberArray() {
        assertTrue(SmartDashboardLogger.debugNumberArray(nextKey(), TEST_NUMBER_ARRAY));
    }

    @Test
    void infoNumberArray() {
        assertTrue(SmartDashboardLogger.infoNumberArray(nextKey(), TEST_NUMBER_ARRAY));
    }

    @Test
    void warnNumberArray() {
        assertTrue(SmartDashboardLogger.warnNumberArray(nextKey(), TEST_NUMBER_ARRAY));
    }

    @Test
    void errorNumberArray() {
        assertTrue(SmartDashboardLogger.errorNumberArray(nextKey(), TEST_NUMBER_ARRAY));
    }

    @Test
    void errorNumberArray_originally_string() {
        final String key = nextKey();
        assertTrue(SmartDashboardLogger.errorString(key, TEST_STRING));
        assertFalse(SmartDashboardLogger.errorNumberArray(key, TEST_NUMBER_ARRAY));
    }

    @Test
    void debugNumber() {
        assertTrue(SmartDashboardLogger.debugNumber(nextKey(), TEST_NUMBER));
    }

    @Test
    void infoNumber() {
        assertTrue(SmartDashboardLogger.infoNumber(nextKey(), TEST_NUMBER));
    }

    @Test
    void warnNumber() {
        assertTrue(SmartDashboardLogger.warnNumber(nextKey(), TEST_NUMBER));
    }

    @Test
    void errorNumber() {
        assertTrue(SmartDashboardLogger.errorNumber(nextKey(), TEST_NUMBER));
    }

    @Test
    void errorNumber_originally_string() {
        final String key = nextKey();
        assertTrue(SmartDashboardLogger.errorString(key, TEST_STRING));
        assertFalse(SmartDashboardLogger.errorNumber(key, TEST_NUMBER));
    }

    @Test
    void debugString() {
        assertTrue(SmartDashboardLogger.debugString(nextKey(), TEST_STRING));
    }

    @Test
    void infoString() {
        assertTrue(SmartDashboardLogger.infoString(nextKey(), TEST_STRING));
    }

    @Test
    void warnString() {
        assertTrue(SmartDashboardLogger.warnString(nextKey(), TEST_STRING));
    }

    @Test
    void errorString() {
        assertTrue(SmartDashboardLogger.errorString(nextKey(), TEST_STRING));
    }

    @Test
    void errorString_originally_number() {
        final String key = nextKey();
        assertTrue(SmartDashboardLogger.errorNumber(key, 0.125));
        assertFalse(SmartDashboardLogger.errorString(key, TEST_STRING));
    }

    private static String nextKey() {
        return "key" + (++keyIndex);
    }

}
