# 网络配置指南

## 主从机模式说明

ROS系统采用主从机架构，需要正确配置网络才能实现通信。

### 主机模式（Master）

适用于控制电脑，作为 ROS Master 运行。

#### 配置步骤
1. 在网络设置面板选择"主机模式 (Master)"
2. 从下拉列表选择本机IP地址
   - 自动检测所有可用网络接口
   - 建议使用有线网络接口
3. ROS_MASTER_URI 将自动设置为 `http://<本机IP>:11311`
4. 点击"测试连接"验证设置

#### 环境变量
```bash
export ROS_MASTER_URI=http://<本机IP>:11311
export ROS_HOSTNAME=<本机IP>
```

### 从机模式（Slave）

适用于机器人端，需要连接到 ROS Master。

#### 配置步骤
1. 选择"从机模式 (Slave)"
2. 设置主控电脑（Master）的IP地址
3. 选择机器人本机的IP地址
4. 确保与主机在同一网段
5. 测试连接

#### 环境变量
```bash
export ROS_MASTER_URI=http://<主机IP>:11311
export ROS_HOSTNAME=<从机IP>
```

## 网络检查

### 连接测试
1. 主机端：
```bash
roscore
```

2. 从机端：
```bash
rostopic list  # 应该能看到话题列表
```

### 常见问题

1. 连接失败
- 检查网络连接
- 确认IP地址正确
- 检查防火墙设置
- 验证端口11311是否开放

2. 话题通信问题
- 检查ROS_HOSTNAME设置
- 确保主从机能互相ping通
- 检查网络延迟

3. 数据传输缓慢
- 考虑使用有线网络
- 减少传输数据量
- 调整发布频率

## 多机器人配置

### 网络拓扑
```
Master (控制电脑)
    │
    ├── Robot1 (从机1)
    ├── Robot2 (从机2)
    └── Robot3 (从机3)
```

### 配置示例
1. Master:
```bash
ROS_MASTER_URI=http://192.168.1.100:11311
ROS_HOSTNAME=192.168.1.100
```

2. Robot1:
```bash
ROS_MASTER_URI=http://192.168.1.100:11311
ROS_HOSTNAME=192.168.1.101
```

3. Robot2:
```bash
ROS_MASTER_URI=http://192.168.1.100:11311
ROS_HOSTNAME=192.168.1.102
```

## 安全建议

1. 网络安全
- 使用专用网络
- 配置防火墙规则
- 限制端口访问

2. 数据安全
- 加密重要数据
- 定期备份配置
- 监控异常连接

## 故障排除

### 诊断工具
```bash
# 检查网络连接
ping <目标IP>

# 检查ROS连接
rostopic echo /rosout

# 查看网络接口
ifconfig
```

### 日志分析
- 查看 ROS 日志
- 检查系统日志
- 分析网络流量

### 常见错误码
- E001: 无法连接到Master
- E002: 话题发布失败
- E003: 网络超时
- E004: 权限不足 