class PIDController:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None, integral_min=None, integral_max=None):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数

        self.error = 0  # 当前误差
        self.integral = 0  # 积分项
        self.previous_error = 0  # 上一次的误差

        self.output_min = output_min  # 输出最小值
        self.output_max = output_max  # 输出最大值

        self.integral_min = integral_min  # 积分项最小值
        self.integral_max = integral_max  # 积分项最大值

    def update(self, setpoint, process_variable):
        # 计算当前误差
        self.error = setpoint - process_variable

        # 计算积分项
        self.integral += self.error

        # 应用积分项的上下限
        if self.integral_min is not None and self.integral < self.integral_min:
            self.integral = self.integral_min
        if self.integral_max is not None and self.integral > self.integral_max:
            self.integral = self.integral_max

        # 计算微分项
        derivative = self.error - self.previous_error

        # 计算输出
        output = (self.kp * self.error) + (self.ki * self.integral) + (self.kd * derivative)

        # 应用输出限制
        if self.output_min is not None and output < self.output_min:
            output = self.output_min
        if self.output_max is not None and output > self.output_max:
            output = self.output_max

        # 更新上一次的误差
        self.previous_error = self.error

        return output
