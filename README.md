# STM32L4xx_RS-485

## Hardware
![alt text](https://github.com/ilhamahendra14/STM32L4xx_RS-485/blob/106a748552bcd0f5e74845a2eb5825abb7b28c9f/MAX3485_schematic.png?raw=true)
+ I used MAX3485 to read sensor data with RS-485 interface.
+ RE & DE pins (MAX3485) connected with PB1 pin (STM32)

## Firmware
+ I use 'HAL_UARTEx_RxEventCallback' to receive data from the sensors.
  ```C
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    uint8_t *RxDataArr[] = {RxData, RxData1, RxData2, RxData3, RxData4, RxData5, RxData6};
    uint8_t *DataArr[] = {Do, Temp, Sat, Ph, Ec, Sal, Orp};
    for (int i = 0; i < 7; i++) {
      DataArr[i][0] = RxDataArr[i][3];
      DataArr[i][1] = RxDataArr[i][4];
      DataArr[i][2] = RxDataArr[i][5];
      DataArr[i][3] = RxDataArr[i][6];
    }
  }
  ```  
+ CRC will calculate automatically.
  ```C
  uint16_t crc = crc16(TxData, 6);
  TxData[6] = crc & 0xFF;   // CRC LOW
  TxData[7] = (crc >> 8) & 0xFF;  // CRC HIGH
  ```
+ Define sensor type first. make sure sensorType, RxData, and Value are in the correct order.
  ```C
  uint8_t* RxDataArr[] = {RxData, RxData1, RxData2, RxData3, RxData4, RxData5, RxData6};
  uint8_t* dataValArr[] = {Do, Temp, Sat, Ph, Ec, Sal, Orp};
  uint8_t* dataFrameArr[] = {&slaveAddr, &functCode, &startAddr1, &startAddr2, &dataLen1, &dataLen2};
  ```
+ If each sensor has a different data type, please edit it in 'HAL_UARTEx_RxEventCallback' function.
  ```C
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    uint8_t *RxDataArr[] = {RxData, RxData1, RxData2, RxData3, RxData4, RxData5, RxData6};
    uint8_t *DataArr[] = {Do, Temp, Sat, Ph, Ec, Sal, Orp};
    for (int i = 0; i < 7; i++) {
      if(i==1){
        ...
      }
      else if(i==3){
        ...
      }
      else{
        DataArr[i][0] = RxDataArr[i][3];
        DataArr[i][1] = RxDataArr[i][4];
        DataArr[i][2] = RxDataArr[i][5];
        DataArr[i][3] = RxDataArr[i][6];
      }
    }
  }
  ```  