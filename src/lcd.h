/*
 * lcd.h
 *
 *  Created on: 16Oct.,2019
 *      Author: s3601377
 */

#ifndef LCD_H_
#define LCD_H_
#define DATA_SEND 0x40  // sets the Rs value high
#define Co_Ctrl   0x00  // mode to tell LCD we are sending a single command


// Function prototypes
int main(int argc, char *argv[]);
int  I2cWrite_(int fd, uint8_t Address, uint8_t mode, uint8_t *pBuffer, uint32_t NbData);
void SetCursor(int fd, uint8_t LCDi2cAdd, uint8_t row, uint8_t column);
void Initialise_LCD (int fd, _Uint32t LCDi2cAdd);



/****************************************************LCD CODE********************************/
// Writes to I2C
int  I2cWrite_(int fd, uint8_t Address, uint8_t mode, uint8_t *pBuffer, uint32_t NbData)
{
    i2c_send_t hdr;
    iov_t sv[2];
    int status, i;

    uint8_t LCDpacket[21] = {};  // limited to 21 characters  (1 control bit + 20 bytes)

    // set the mode for the write (control or data)
    LCDpacket[0] = mode;  // set the mode (data or control)

    // copy data to send to send buffer (after the mode bit)
    for (i=0;i<NbData+1;i++)
        LCDpacket[i+1] = *pBuffer++;

    hdr.slave.addr = Address;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.len = NbData + 1;  // 1 extra for control (mode) bit
    hdr.stop = 1;

    SETIOV(&sv[0], &hdr, sizeof(hdr));
    SETIOV(&sv[1], &LCDpacket[0], NbData + 1); // 1 extra for control (mode) bit
      // int devctlv(int filedes, int dcmd,     int sparts, int rparts, const iov_t *sv, const iov_t *rv, int *dev_info_ptr);
    status = devctlv(fd,          DCMD_I2C_SEND, 2,          0,          sv,              NULL,           NULL);

    if (status != EOK)
        printf("status = %s\n", strerror ( status ));

    return status;
}


void SetCursor(int fd, uint8_t LCDi2cAdd, uint8_t row, uint8_t column)
{
    uint8_t position = 0x80; // SET_DDRAM_CMD (control bit)
    uint8_t rowValue = 0;
    uint8_t LCDcontrol = 0;
    if (row == 1)
        rowValue = 0x40;     // memory location offset for row 1
    position = (uint8_t)(position + rowValue + column);
    LCDcontrol = position;
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C
}


void Initialise_LCD (int fd, _Uint32t LCDi2cAdd)
{
    uint8_t LCDcontrol = 0x00;

    //   Initialise the LCD display via the I2C bus
    LCDcontrol = 0x38;  // data byte for FUNC_SET_TBL1
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x39; // data byte for FUNC_SET_TBL2
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x14; // data byte for Internal OSC frequency
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x79; // data byte for contrast setting
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x50; // data byte for Power/ICON control Contrast set
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x6C; // data byte for Follower control
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x0C; // data byte for Display ON
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C

    LCDcontrol = 0x01; // data byte for Clear display
    I2cWrite_(fd, LCDi2cAdd, Co_Ctrl, &LCDcontrol, 1);      // write data to I2C
}






#endif /* LCD_H_ */
