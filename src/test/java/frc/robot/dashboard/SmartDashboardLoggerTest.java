package frc.robot.dashboard;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SmartDashboardLoggerTest {

    private static final String TEST_STRING = "test value";

    private static final double TEST_NUMBER = 0.125;

    @Test
    void putNumber() {
        assertTrue(SmartDashboardLogger.putNumber("key1", TEST_NUMBER));
    }

    @Test
    void putNumber_originally_string() {
        assertTrue(SmartDashboardLogger.putString("key2", TEST_STRING));
        assertFalse(SmartDashboardLogger.putNumber("key2", TEST_NUMBER));
    }

    @Test
    void putString() {
        assertTrue(SmartDashboardLogger.putString("key3", TEST_STRING));
    }

    @Test
    void putString_originally_number() {
        assertTrue(SmartDashboardLogger.putNumber("key4", 0.125));
        assertFalse(SmartDashboardLogger.putString("key4", TEST_STRING));
    }
}
