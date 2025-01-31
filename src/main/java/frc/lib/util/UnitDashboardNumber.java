package frc.lib.util;

public class UnitDashboardNumber extends LoggedTunableNumber {
    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public UnitDashboardNumber(String dashboardKey) {
       super(dashboardKey, 0);
    }
}