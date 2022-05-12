from machine import I2C
import time
import math

Gyro  = [0,0,0]
Accel = [0,0,0]
Mag   = [0,0,0]
pitch = 0.0
roll  = 0.0
yaw   = 0.0
pu8data=[0,0,0,0,0,0,0,0]
U8tempX=[0,0,0,0,0,0,0,0,0]
U8tempY=[0,0,0,0,0,0,0,0,0]
U8tempZ=[0,0,0,0,0,0,0,0,0]
GyroOffset=[0,0,0]
Ki = 1.0
Kp = 4.50
q0 = 1.0
q1=q2=q3=0.0
angles=[0.0,0.0,0.0]
true                                 =0x01
false                                =0x00
# define ICM-20948 Device I2C address
I2C_ADD_ICM20948                     = 0x68
I2C_ADD_ICM20948_AK09916             = 0x0C
I2C_ADD_ICM20948_AK09916_READ        = 0x80
I2C_ADD_ICM20948_AK09916_WRITE       = 0x00
# define ICM-20948 Register
# user bank 0 register
REG_ADD_WIA                          = 0x00
REG_VAL_WIA                          = 0xEA
REG_ADD_USER_CTRL                    = 0x03
REG_VAL_BIT_DMP_EN                   = 0x80
REG_VAL_BIT_FIFO_EN                  = 0x40
REG_VAL_BIT_I2C_MST_EN               = 0x20
REG_VAL_BIT_I2C_IF_DIS               = 0x10
REG_VAL_BIT_DMP_RST                  = 0x08
REG_VAL_BIT_DIAMOND_DMP_RST          = 0x04
REG_ADD_PWR_MIGMT_1                  = 0x06
REG_VAL_ALL_RGE_RESET                = 0x80
REG_VAL_RUN_MODE                     = 0x01 # Non low-power mode
REG_ADD_LP_CONFIG                    = 0x05
REG_ADD_PWR_MGMT_1                   = 0x06
REG_ADD_PWR_MGMT_2                   = 0x07
# REG_ADD_ACCEL_XOUT_H                 = 0x2D
# REG_ADD_ACCEL_XOUT_L                 = 0x2E
# REG_ADD_ACCEL_YOUT_H                 = 0x2F
# REG_ADD_ACCEL_YOUT_L                 = 0x30
# REG_ADD_ACCEL_ZOUT_H                 = 0x31
# REG_ADD_ACCEL_ZOUT_L                 = 0x32
# REG_ADD_GYRO_XOUT_H                  = 0x33
# REG_ADD_GYRO_XOUT_L                  = 0x34
# REG_ADD_GYRO_YOUT_H                  = 0x35
# REG_ADD_GYRO_YOUT_L                  = 0x36
# REG_ADD_GYRO_ZOUT_H                  = 0x37
# REG_ADD_GYRO_ZOUT_L                  = 0x38
REG_ADD_EXT_SENS_DATA_00             = 0x3B
REG_ADD_REG_BANK_SEL                 = 0x7F
REG_VAL_REG_BANK_0                   = 0x00
REG_VAL_REG_BANK_1                   = 0x10
REG_VAL_REG_BANK_2                   = 0x20
REG_VAL_REG_BANK_3                   = 0x30

# user bank 1 register
# user bank 2 register
#REG_ADD_GYRO_SMPLRT_DIV              = 0x00
#REG_ADD_GYRO_CONFIG_1                = 0x01
# REG_VAL_BIT_GYRO_DLPCFG_2            = 0x10  # bit[5:3]
# REG_VAL_BIT_GYRO_DLPCFG_4            = 0x20  # bit[5:3]
# REG_VAL_BIT_GYRO_DLPCFG_6            = 0x30  # bit[5:3]
# REG_VAL_BIT_GYRO_FS_250DPS           = 0x00  # bit[2:1]
# REG_VAL_BIT_GYRO_FS_500DPS           = 0x02  # bit[2:1]
# REG_VAL_BIT_GYRO_FS_1000DPS          = 0x04  # bit[2:1]
# REG_VAL_BIT_GYRO_FS_2000DPS          = 0x06  # bit[2:1]
# REG_VAL_BIT_GYRO_DLPF                = 0x01  # bit[0]
# REG_ADD_ACCEL_SMPLRT_DIV_2           = 0x11
# REG_ADD_ACCEL_CONFIG                 = 0x14
# REG_VAL_BIT_ACCEL_DLPCFG_2           = 0x10  # bit[5:3]
# REG_VAL_BIT_ACCEL_DLPCFG_4           = 0x20  # bit[5:3]
# REG_VAL_BIT_ACCEL_DLPCFG_6           = 0x30  # bit[5:3]
# REG_VAL_BIT_ACCEL_FS_2g              = 0x00  # bit[2:1]
# REG_VAL_BIT_ACCEL_FS_4g              = 0x02  # bit[2:1]
# REG_VAL_BIT_ACCEL_FS_8g              = 0x04  # bit[2:1]
# REG_VAL_BIT_ACCEL_FS_16g             = 0x06  # bit[2:1]
# REG_VAL_BIT_ACCEL_DLPF               = 0x01  # bit[0]

# user bank 3 register
REG_ADD_I2C_SLV0_ADDR                = 0x03
REG_ADD_I2C_SLV0_REG                 = 0x04
REG_ADD_I2C_SLV0_CTRL                = 0x05
REG_VAL_BIT_SLV0_EN                  = 0x80
REG_VAL_BIT_MASK_LEN                 = 0x07
REG_ADD_I2C_SLV0_DO                  = 0x06
REG_ADD_I2C_SLV1_ADDR                = 0x07
REG_ADD_I2C_SLV1_REG                 = 0x08
REG_ADD_I2C_SLV1_CTRL                = 0x09
REG_ADD_I2C_SLV1_DO                  = 0x0A

# define ICM-20948 Register  end

# define ICM-20948 MAG Register
REG_ADD_MAG_WIA1                     = 0x00
REG_VAL_MAG_WIA1                     = 0x48
REG_ADD_MAG_WIA2                     = 0x01
REG_VAL_MAG_WIA2                     = 0x09
REG_ADD_MAG_ST2                      = 0x10
REG_ADD_MAG_DATA                     = 0x11
REG_ADD_MAG_CNTL2                    = 0x31
REG_VAL_MAG_MODE_PD                  = 0x00
REG_VAL_MAG_MODE_SM                  = 0x01
REG_VAL_MAG_MODE_10HZ                = 0x02
REG_VAL_MAG_MODE_20HZ                = 0x04
REG_VAL_MAG_MODE_50HZ                = 0x05
REG_VAL_MAG_MODE_100HZ               = 0x08
REG_VAL_MAG_MODE_ST                  = 0x10
# define ICM-20948 MAG Register  end

MAG_DATA_LEN                         =6

class ICM20948(object):
  def __init__(self,address=I2C_ADD_ICM20948):
    self._address = address
    self._bus = I2C(1)
    bRet=self.icm20948Check()             #Initialization of the device multiple times after power on will result in a return error
    # while true != bRet:
    #   print("ICM-20948 Error\n" )
    #   time.sleep(0.5)
    # print("ICM-20948 OK\n" )
    time.sleep(0.5)                       #We can skip this detection by delaying it by 500 milliseconds
    # user bank 0 register 
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0)
    self._write_byte( REG_ADD_PWR_MIGMT_1 , REG_VAL_ALL_RGE_RESET)
    time.sleep(0.1)
    self._write_byte( REG_ADD_PWR_MIGMT_1 , REG_VAL_RUN_MODE)  
    #user bank 2 register
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_2)

    #user bank 0 register
    self._write_byte( REG_ADD_REG_BANK_SEL , REG_VAL_REG_BANK_0) 
    time.sleep(0.1)

    self.icm20948MagCheck()
    self.icm20948WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ)

  
  def icm20948MagRead(self):
    counter=20
    while(counter>0):
      time.sleep(0.01)
      self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ , REG_ADD_MAG_ST2, 1)
      if ((pu8data[0] & 0x01)!= 0):
        break
      counter-=1
    if counter!=0:
      for i in range(0,8):
        self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ , REG_ADD_MAG_DATA , MAG_DATA_LEN)
        U8tempX[i] = (pu8data[1]<<8)|pu8data[0]
        U8tempY[i] = (pu8data[3]<<8)|pu8data[2]
        U8tempZ[i] = (pu8data[5]<<8)|pu8data[4]
      Mag[0]=(U8tempX[0]+U8tempX[1]+U8tempX[2]+U8tempX[3]+U8tempX[4]+U8tempX[5]+U8tempX[6]+U8tempX[7])/8
      Mag[1]=-(U8tempY[0]+U8tempY[1]+U8tempY[2]+U8tempY[3]+U8tempY[4]+U8tempY[5]+U8tempY[6]+U8tempY[7])/8
      Mag[2]=-(U8tempZ[0]+U8tempZ[1]+U8tempZ[2]+U8tempZ[3]+U8tempZ[4]+U8tempZ[5]+U8tempZ[6]+U8tempZ[7])/8
    if Mag[0]>=32767:            #Solve the problem that Python shift will not overflow
      Mag[0]=Mag[0]-65535
    elif Mag[0]<=-32767:
      Mag[0]=Mag[0]+65535
    if Mag[1]>=32767:
      Mag[1]=Mag[1]-65535
    elif Mag[1]<=-32767:
      Mag[1]=Mag[1]+65535
    if Mag[2]>=32767:
      Mag[2]=Mag[2]-65535
    elif Mag[2]<=-32767:
      Mag[2]=Mag[2]+65535
  def icm20948ReadSecondary(self,u8I2CAddr,u8RegAddr,u8Len):
    u8Temp=0
    self._write_byte( REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3) #swtich bank3
    self._write_byte( REG_ADD_I2C_SLV0_ADDR, u8I2CAddr)
    self._write_byte( REG_ADD_I2C_SLV0_REG,  u8RegAddr)
    self._write_byte( REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
    
    u8Temp = self._read_byte(REG_ADD_USER_CTRL)
    u8Temp |= REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)
    time.sleep(0.01)
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)
    
    for i in range(0,u8Len):
      pu8data[i]= self._read_byte( REG_ADD_EXT_SENS_DATA_00+i)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3) #swtich bank3
    
    u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN))
    self._write_byte( REG_ADD_I2C_SLV0_CTRL,  u8Temp)
    
    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0
  def icm20948WriteSecondary(self,u8I2CAddr,u8RegAddr,u8data):
    u8Temp=0
    self._write_byte( REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3) #swtich bank3
    self._write_byte( REG_ADD_I2C_SLV1_ADDR, u8I2CAddr)
    self._write_byte( REG_ADD_I2C_SLV1_REG,  u8RegAddr)
    self._write_byte( REG_ADD_I2C_SLV1_DO,   u8data)
    self._write_byte( REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0

    u8Temp = self._read_byte(REG_ADD_USER_CTRL)
    u8Temp |= REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)
    time.sleep(0.01)
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN
    self._write_byte( REG_ADD_USER_CTRL, u8Temp)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3) #swtich bank3

    u8Temp = self._read_byte(REG_ADD_I2C_SLV0_CTRL)
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN))
    self._write_byte( REG_ADD_I2C_SLV0_CTRL,  u8Temp)

    self._write_byte( REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0) #swtich bank0

  def _read_byte(self,cmd):
    rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
    return rec[0]
  def _read_block(self, reg, length=1):
    rec=self._bus.readfrom_mem(int(self._address),int(reg),length)
    return rec
  def _read_u16(self,cmd):
    LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
    MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
    return (MSB[0] << 8) + LSB[0]

  def _write_byte(self,cmd,val):
    self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
    time.sleep(0.0001)

  def icm20948Check(self):
    bRet=false
    if REG_VAL_WIA == self._read_byte(REG_ADD_WIA):
      bRet = true
    return bRet
  def icm20948MagCheck(self):
    self.icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,REG_ADD_MAG_WIA1, 2)
    if (pu8data[0] == REG_VAL_MAG_WIA1) and ( pu8data[1] == REG_VAL_MAG_WIA2) :
        bRet = true
        return bRet

if __name__ == '__main__':
  import time
  print("\n3-axis Hall_Sensor ...\n")
  
  icm20948=ICM20948()
  while True:
    icm20948.icm20948MagRead()
    time.sleep(0.1)
   
    print(Mag[0] ,Mag[1], Mag[2])
    



       
      
         
      





      

