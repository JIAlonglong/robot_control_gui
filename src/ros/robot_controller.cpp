void RobotController::enableAutoLocalization(bool enable)
{
    auto_localization_enabled_ = enable;
    
    if (enable) {
        // 订阅定位质量评估话题
        localization_quality_sub_ = nh_.subscribe<std_msgs::Float32>(
            "localization_quality", 1,
            [this](const std_msgs::Float32::ConstPtr& msg) {
                if (msg->data < 0.8) { // 定位质量阈值
                    // 触发自动定位纠正,但不中断导航
                    startAutoLocalization();
                }
            });
    } else {
        // 取消订阅
        localization_quality_sub_.shutdown();
    }
} 