std_msgs/Header header

# 基座姿态与运动学
BaseState base_state

# 四条腿的状态: [0]:LF, [1]:LH, [2]:RF, [3]:RH
LegState[4] leg_state

# 系统性能监控
SystemMonitor system_monitor