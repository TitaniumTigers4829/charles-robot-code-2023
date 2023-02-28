package frc.robot.dashboard;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SmartDashboardLoggerTest {

    private static final String TEST_STRING = "test value";

    private static final double TEST_NUMBER = 0.125;

    private static int keyIndex = 0;

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
