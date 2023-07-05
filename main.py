import machine
import time
import struct
import network
import config

# 初始化I2C总线
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4))

# BMP280设备地址
BMP280_ADDRESS = 0x77
# BMP280寄存器地址
BMP280_REGISTER_ID = 0xD0
BMP280_REGISTER_CTRL_MEAS = 0xF4
BMP280_REGISTER_CONFIG = 0xF5
BMP280_REGISTER_TEMPDATA = 0xFA
BMP280_REGISTER_PRESSUREDATA = 0xF7

# AHT20设备地址
AHT20_ADDRESS = 0x38

# AHT20寄存器地址
AHT20_MEASURE_CMD = bytearray([0xAC, 0x33, 0x00])

# 海平面气压
sea_level_pressure = 1013.25

# 气压计校准参数
dig_T1 = None
dig_T2 = None
dig_T3 = None
dig_P1 = None
dig_P2 = None
dig_P3 = None
dig_P4 = None
dig_P5 = None
dig_P6 = None
dig_P7 = None
dig_P8 = None
dig_P9 = None

# 读取BMP280传感器ID
def read_bmp280_id():
    data = i2c.readfrom_mem(BMP280_ADDRESS, BMP280_REGISTER_ID, 1)
    return data[0]

# 读取BMP280传感器校准参数
def read_bmp280_calib_params():
    global dig_T1, dig_T2, dig_T3
    global dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9
    
    data = i2c.readfrom_mem(BMP280_ADDRESS, 0x88, 24)
    dig_T1, dig_T2, dig_T3 = struct.unpack("<Hhh", data[0:6])
    dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9 = struct.unpack("<Hhhhhhhhh", data[6:24])

# 读取BMP280传感器温度
def read_bmp280_temperature():
    global t_fine
    
    # 开始温度测量
    i2c.writeto_mem(BMP280_ADDRESS, BMP280_REGISTER_CTRL_MEAS, bytearray([0x2E]))
    time.sleep(0.005)
    
    # 读取温度数据
    data = i2c.readfrom_mem(BMP280_ADDRESS, BMP280_REGISTER_TEMPDATA, 3)
    adc_T = (data[0] << 16 | data[1] << 8 | data[2]) >> 4

    # 计算温度
    var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11
    var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14
    t_fine = var1 + var2
    temperature = (t_fine * 5 + 128) >> 8
    
    return temperature / 100.0

# 读取BMP280传感器气压
def read_bmp280_pressure():
    global t_fine
    
    # 开始气压测量
    i2c.writeto_mem(BMP280_ADDRESS, BMP280_REGISTER_CTRL_MEAS, bytearray([0x34]))
    time.sleep(0.005)
    
    # 读取气压数据
    data = i2c.readfrom_mem(BMP280_ADDRESS, BMP280_REGISTER_PRESSUREDATA, 3)
    adc_P = (data[0] << 16 | data[1] << 8 | data[2]) >> 4

    # 计算气压
    var1 = t_fine - 128000
    var2 = var1 * var1 * dig_P6
    var2 = var2 + ((var1 * dig_P5) << 17)
    var2 = var2 + (dig_P4 << 35)
    var1 = ((var1 * var1 * dig_P3) >> 8) + ((var1 * dig_P2) << 12)
    var1 = (((1 << 47) + var1)) * dig_P1 >> 33
    if var1 == 0:
        return 0
    p = 1048576 - adc_P
    p = (((p << 31) - var2) * 3125) // var1
    var1 = (dig_P9 * (p >> 13) * (p >> 13)) >> 25
    var2 = (dig_P8 * p) >> 19
    p = ((p + var1 + var2) >> 8) + (dig_P7 << 4)
    
    return p / 25600.0

# 读取AHT20传感器温度和湿度
def read_aht20_data():
    # 发送测量命令
    i2c.writeto(AHT20_ADDRESS, AHT20_MEASURE_CMD)
    time.sleep(0.1)
    
    # 读取温度和湿度数据
    data = i2c.readfrom(AHT20_ADDRESS, 6)
    raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
    raw_humidity = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
    
    # 计算温度和湿度
    temperature = ((raw_temperature / 1048576.0) * 200) - 50
    humidity = ((raw_humidity / 1048576.0) * 100)
    
    return temperature, humidity

# 读取传感器数据
def read_sensor_data():
    temperature_bmp_sum = 0
    pressure_sum = 0
    temperature_aht_sum = 0
    humidity_sum = 0
    
    for i in range(4):
        temperature_bmp, pressure = read_bmp280_data()
        temperature_aht, humidity = read_aht20_data()
        
        temperature_bmp_sum += temperature_bmp
        pressure_sum += pressure
        temperature_aht_sum += temperature_aht
        humidity_sum += humidity
        
    temperature_bmp_avg = temperature_bmp_sum / 4
    pressure_avg = pressure_sum / 4
    temperature_aht_avg = temperature_aht_sum / 4
    humidity_avg = humidity_sum / 4
    
    return temperature_bmp_avg, pressure_avg, temperature_aht_avg, humidity_avg


# 读取BMP280和AHT20传感器数据
def read_bmp280_data():
    global t_fine
    
    temperature = read_bmp280_temperature()
    pressure = read_bmp280_pressure()
    return temperature, pressure

# 读取传感器ID和校准参数
read_bmp280_id()
read_bmp280_calib_params()

#关闭LED
led1=machine.Pin(12,machine.Pin.OUT)
led2=machine.Pin(13,machine.Pin.OUT)
led1.off()
led2.off()


#连接WiFi

# WiFi连接配置
SSID = config.ssid
PASSWORD = config.password

try:
    # 初始化WiFi连接
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
except:
    wlan = network.WLAN(network.STA_IF)
    wlan.disconnect()
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)

# 等待WiFi连接成功
while not wlan.isconnected():
    time.sleep(1)
    pass

# 连接成功后打印IP地址
print("WiFi connected successfully")
print("IP address:", wlan.ifconfig()[0])
led1.on()
time.sleep(1)
led1.off()

# 主循环
while True:
    # 读取BMP280和AHT20传感器数据
    temperature_bmp, pressure, temperature_aht, humidity = read_sensor_data()
    # 处理传感器数据
    print("BMP280: temperature = %.2f C, pressure = %.2f kPa" % (temperature_bmp, pressure))
    print("AHT20: temperature = %.2f C, humidity = %.2f %%" % (temperature_aht, humidity))
    altitude = ((sea_level_pressure / pressure) ** (1 / 5.257) - 1) * (temperature_aht + 273.15) / 0.0065
    print("Altitude = %.2f m" % altitude)
    time.sleep(1)
