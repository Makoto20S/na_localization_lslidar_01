# ros 时间
Header header

# GPS 周
uint16 week
# 从 GPS 周起算的毫秒
uint32 ms

# 解状态
# 0 SOL_COMPUTED 解算成功
# 1 INSUFFICIENT_OBS 观测量不足
# 6 COLD_START 冷启动中
# 19 INVALID_FIX Fix position 输入的坐标误差超限， 将停止差分数据的发送
uint32 sol_stat

# 定位类型
# 0 NONE 未解算
# 1 FIXEDPOS 通过指令 FIX POSITION 进行位置固定
# 8 DOPPLER_VELOCITY 利用实时多普勒计算速度
# 9 SINGLE_SMOOTH 单点平滑定位
# 16 SINGLE 单点定位
# 17 PSRDIFF 伪距差分定位
# 18 SBAS 加入 SBAS 改正的解
# 34 NARROW_FLOAT 浮点解
# 35 FIX_DERIVATION 推导解
# 49 WIDE_INT 宽巷解
# 50 NARROW_INT 固定解
# 51 SUPER WIDE_LANE 超宽巷解
# 64 OMNISTAR_HP OmniSTAR HP 定位解
# 65 OMNISTAR_XP OmniSTAR XP 定位解
# 68 PPP_CONVERGING PPP 收敛解
# 69 PPP PPP 浮点解
# 70 OPERATIONAL 根据用户定义的标准差阈值设置输出， 在米级内输出
# 71 WARNING 解算精度超出用户设定的阈值， 但在预警范围内
# 72 OUT_OF_BOUNDS 超出用户定义的标准差阈值
uint32 pos_type

# 纬度
float64 lat
# 经度
float64 lon
# 高度（以平均海平面为起算面）
float32 hgt
# 高程异常值， 大地水准面和WGS84参考椭球面的差值（m）
float32 undulation

# 纬度标准差
float32 lat_std_dev
# 经度标准差
float32 lon_std_dev
# 高度标准差
float32 hgt_std_dev

# 跟踪的卫星数
uint8 svs
# 参与解算的卫星数
uint8 solnsvs



