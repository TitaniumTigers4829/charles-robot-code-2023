package frc.robot.extras;

public enum SmartDashboardLogLevel {

    DEBUG(16), INFO(32), WARN(48), ERROR(64);

    private final int level;

    SmartDashboardLogLevel(final int level) {
        this.level = level;
    }

    public int getLevel() {
        return level;
    }
}
