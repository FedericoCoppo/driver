/*
 * IS25LP016D.h
 *
 *  Created on: 17-03-2020
 *      Author: FCoppo
 */

/*******************************************************************************
    Includes
*******************************************************************************/

#include <string.h>
#include "IS25LP016D.h"
#include "HL_mibspi_LIBRA.h"

/*******************************************************************************
    Define
*******************************************************************************/

 /* IS25LP016D address map */
#define IS25LP016D_MAP_START_ADDRESS                        0x00000                           /* start address of E2prom */
#define IS25LP016D_BLOCK4k_BYTE_SIZE                        0x01000                           /* 4Kbyte in 1 block of E2prom */
#define IS25LP016D_BLOCK32k_BYTE_SIZE                       0x08000                           /* 32Kbyte in 1 block of E2prom */
#define IS25LP016D_BLOCK64k_BYTE_SIZE                       0x10000                           /* 64Kbyte in 1 block of E2prom */
#define IS25LP016D_END_MEM_ADD                              0x001FFFFF
#define IS25LP016D_BYTE_SIZE                                0x200000

 /* Status Register */
 #define IS25LP016D_SR_WIP                               ((uint8_t)0x01) /*!< Write in progress */
 #define IS25LP016D_SR_WREN                              ((uint8_t)0x02) /*!< Write enable latch */
 #define IS25LP016D_SR_BLOCKPR                           ((uint8_t)0x3C) /*!< Block protected against program and erase operations */
 #define IS25LP016D_SR_QUADEN                            ((uint8_t)0x40) /*!< Quad IO mode enabled if =1 */
 #define IS25LP016D_SR_SWRD                              ((uint8_t)0x80) /*!< Status register write enable/disable */

 /* device parameters */
 #define IS25LP016D_MANUFACTURER                         (0x9d)
 #define IS25LP016D_MEM_TYPE_CAPACITY                    (0x6015)
 #define IS25LP016D_CAPACITY                             (0x15)
 #define IS25LP016D_DEV_ID                               (0x14)

 /* < commands > */

 /* Power Down Mode */
 #define IS25LP016D_ENTER_DEEP_POWER_DOWN_CMD            (0xB9)
 #define IS25LP016D_RELEASE_DEEP_POWER_DOWN_CMD          (0xAB)

 /* Reset Operations */
 #define IS25LP016D_SW_RESET_ENABLE_CMD                  (0x66)
 #define IS25LP016D_RESET_ENABLE_CMD                     (0x99)

 /* Identification Operations */
 #define IS25LP016D_READ_MEM_CAP_CMD                     (0x9F)
 #define IS25LP016D_READ_DEV_ID_CMD                      (0xAB)
 #define IS25LP016D_READ_MAN_ID                          (0x90)

 /* Read Operations */
 #define IS25LP016D_READ_NORMALMODE_CMD                  (0x03)

 /* Write Operations */
 #define IS25LP016D_WRITE_ENABLE_CMD                     (0x06)
 #define IS25LP016D_WRITE_DISABLE_CMD                    (0x04)

 /* Register Operations */
 #define IS25LP016D_READ_STATUS_REG_CMD                  (0x05)
 #define IS25LP016D_WRITE_STATUS_REG_CMD                 (0x01)

 #define IS25LP016D_READ_EXTSTATUS_REG_CMD               (0x81)
#define IS25LP016D_CLEAR_EXTSTATUS_REG_CMD               (0x82)


 /* Program Operations */
 #define IS25LP016D_PAGE_PROG_CMD                        (0x02)

 /* Erase Operations */
 #define IS25LP016D_BLOCK_4_ERASE_CMD                    (0x20)
 #define IS25LP016D_BLOCK_32_ERASE_CMD                   (0x52)
 #define IS25LP016D_BLOCK_64_ERASE_CMD                   (0xD8)
 #define IS25LP016D_CHIP_ERASE_CMD                       (0xC7)

 #define IS25LP016D_PROG_ERASE_SUSPEND_CMD               (0x75)
 #define IS25LP016D_PROG_ERASE_RESUME_CMD                (0x7A)

 /* One-Time Programmable Operations */
 #define IS25LP016D_READ_UNIQUE_ID_CMD                   (0x4B)
 #define PROG_OTP_ARRAY_CMD                              (0x42)

/* MIBSPI group */
#define IS25LP016D_WRITE_READ_16BIT_64WORD                  0
#define IS25LP016D_WRITE_ENBL_8BIT_1WORD                    1
#define IS25LP016D_WRITE_READ_16BIT_1WORD                   2
#define IS25LP016D_READ_ID_16BIT_3WORD                      3
#define IS25LP016D_ERASE_16BIT_2WORD                        4

#define IS25LP016D_SESSION_DATA_WORD_SIZE                 0x40u
#define IS25LP016D_BUFFER_WORD_SIZE                       IS25LP016D_SESSION_DATA_WORD_SIZE + 2

 /*******************************************************************************
     Enum
 *******************************************************************************/

 typedef enum
 {
     /* Read  */
     ReadStatusReg,
     ReadExtReadReg,

     /* Read Device info  */
     ReadDeviceId,
     ReadManId,
     ReadMemType,
     ReadDataNormal,

     /* write */
     WriteEnbl,
     WriteStatusReg,
     WriteData,

     /* Erase Operation */
     EraseBlock64k,
     EraseBlock32k,
     EraseBlock4k,
     EraseChip,

     /* operation command */
     ClearExtReadReg,
     SwResetEnbl,
     SwReset,
     EnterPwDownMode,
     ReleasePwDownMode,

     CommandSize,

 } IS25LP016D_Cmd_E;

 /* Command struct */
 typedef struct IS25LP016D_Command_Struct
 {
     uint8  id;
     uint8  cmd;
     uint8  mibSPIGroup;
 } IS25LP016D_cmd_struct;


 static const IS25LP016D_cmd_struct cmd[CommandSize]=
 {

  {ReadStatusReg,    {IS25LP016D_READ_STATUS_REG_CMD},           IS25LP016D_WRITE_READ_16BIT_1WORD},
  {ReadExtReadReg,   {IS25LP016D_READ_EXTSTATUS_REG_CMD},        IS25LP016D_WRITE_READ_16BIT_1WORD},

  {ReadDeviceId,     {IS25LP016D_READ_DEV_ID_CMD},               IS25LP016D_READ_ID_16BIT_3WORD},
  {ReadManId,        {IS25LP016D_READ_MAN_ID},                   IS25LP016D_READ_ID_16BIT_3WORD},
  {ReadMemType,      {IS25LP016D_READ_MEM_CAP_CMD},              IS25LP016D_READ_ID_16BIT_3WORD},

  {ReadDataNormal,   {IS25LP016D_READ_NORMALMODE_CMD},           IS25LP016D_WRITE_READ_16BIT_64WORD},

  {WriteEnbl,        {IS25LP016D_WRITE_ENABLE_CMD},              IS25LP016D_WRITE_ENBL_8BIT_1WORD},
  {WriteStatusReg,   {IS25LP016D_WRITE_STATUS_REG_CMD},          IS25LP016D_WRITE_READ_16BIT_1WORD},
  {WriteData,       {IS25LP016D_PAGE_PROG_CMD},                 IS25LP016D_WRITE_READ_16BIT_64WORD},

  {EraseBlock64k,    {IS25LP016D_BLOCK_64_ERASE_CMD},            IS25LP016D_ERASE_16BIT_2WORD},
  {EraseBlock32k,    {IS25LP016D_BLOCK_32_ERASE_CMD},            IS25LP016D_ERASE_16BIT_2WORD},
  {EraseBlock4k,     {IS25LP016D_BLOCK_4_ERASE_CMD},             IS25LP016D_ERASE_16BIT_2WORD},
  {EraseChip,        {IS25LP016D_CHIP_ERASE_CMD},                IS25LP016D_WRITE_ENBL_8BIT_1WORD},

  {ClearExtReadReg,  {IS25LP016D_CLEAR_EXTSTATUS_REG_CMD},       IS25LP016D_WRITE_ENBL_8BIT_1WORD},
  {SwResetEnbl,      {IS25LP016D_SW_RESET_ENABLE_CMD},           IS25LP016D_WRITE_ENBL_8BIT_1WORD},
  {SwReset,          {IS25LP016D_RESET_ENABLE_CMD},              IS25LP016D_WRITE_ENBL_8BIT_1WORD},
  {EnterPwDownMode,  {IS25LP016D_ENTER_DEEP_POWER_DOWN_CMD},     IS25LP016D_WRITE_ENBL_8BIT_1WORD},
  {ReleasePwDownMode,{IS25LP016D_RELEASE_DEEP_POWER_DOWN_CMD},   IS25LP016D_WRITE_ENBL_8BIT_1WORD},
 };


/*******************************************************************************
    Module variables definition
*******************************************************************************/

/*******************************************************************************
    Public functions definition
*******************************************************************************/
static uint16_t IS25LP016D_internalBuff[IS25LP016D_BUFFER_WORD_SIZE] = {0};

/* Blocking Transmit routine */
static uint8_t IS25LP016D_Transmit (IS25LP016D_Cmd_E index, uint16_t* tx)
{
    uint8_t spiDelay = 255;
    mibspiSetData(mibspiREG5, cmd[index].mibSPIGroup, tx);
    mibspiTransfer(mibspiREG5, cmd[index].mibSPIGroup);
    while ( (!mibspiIsTransferComplete(mibspiREG5, cmd[index].mibSPIGroup)) && (spiDelay > 0u) )  { spiDelay--; }

    return spiDelay;
}

/* Blocking Transmit routine for longer transition */
static uint16_t IS25LP016D_TransmitLong (IS25LP016D_Cmd_E index, uint16_t* tx)
{
    uint16_t spiDelay = 20000;
    mibspiSetData(mibspiREG5, cmd[index].mibSPIGroup, tx);
    mibspiTransfer(mibspiREG5, cmd[index].mibSPIGroup);
    while ( (!mibspiIsTransferComplete(mibspiREG5, cmd[index].mibSPIGroup)) && (spiDelay > 0u) )  { spiDelay--; }

    return spiDelay;
}

/* Read status register */
static boolean IS25LP016D_ReadStatusRegister (uint8* st)
{
    bool res = FALSE;

    /* command */
    uint16_t tx = cmd[ReadStatusReg].cmd;
    tx = (tx << 8);

    uint8_t spiDelay = IS25LP016D_Transmit(ReadStatusReg, &tx);

    if(spiDelay > 0)
    {
        mibspiGetData(mibspiREG5, cmd[ReadStatusReg].mibSPIGroup, &tx);
        *st = (uint8_t) (tx & 0x00FF);
        res = TRUE;
    }

    return res;
}

/* Read "extended read" status register */
static boolean IS25LP016D_ReadExtendedStatusReg (uint8* st)
{
    bool res = FALSE;

    /* command */
    uint16_t tx = cmd[ReadExtReadReg].cmd;
    tx = (tx << 8);

    uint8_t spiDelay = IS25LP016D_Transmit(ReadExtReadReg, &tx);

    if(spiDelay > 0)
    {
        mibspiGetData(mibspiREG5, cmd[ReadExtReadReg].mibSPIGroup, &tx);
        *st = (uint8_t) (tx & 0x00FF);
        res = TRUE;
    }

    return res;
}

/* Read device ID*/
static bool IS25LP016D_ReadDevID (uint8* devId)
{
    bool res = FALSE;

    uint16_t tx[3] = {0};
    tx[0] = cmd[ReadDeviceId].cmd;
    tx[0] = (tx[0] << 8);

    uint8_t spiDelay = IS25LP016D_Transmit(ReadDeviceId, tx);

    if(spiDelay > 0)
    {
        mibspiGetData(mibspiREG5, cmd[ReadDeviceId].mibSPIGroup, tx);
        *devId = (uint8_t) (tx[2] & 0xFF);
        res = TRUE;
    }

    return res;
}

/* Read device Manufacture ID*/
static bool IS25LP016D_ReadManID (uint8* manId)
{
    bool res = FALSE;
    uint16_t tx[3] = {0};

    tx[0] = cmd[ReadManId].cmd;
    tx[0] = (tx[0] << 8);

    uint8_t spiDelay = IS25LP016D_Transmit(ReadManId, tx);

    if(spiDelay > 0)
    {
        mibspiGetData(mibspiREG5, cmd[ReadManId].mibSPIGroup, tx);
        *manId =  (uint8_t) ((tx[2] & 0xFF00) >> 8);
        res = TRUE;
    }

    return res;
}

/* Read device Mem type */
static bool IS25LP016D_ReadMemType (uint16 * memType)
{
    bool res = FALSE;
    uint16_t tx[3] = {0};

    tx[0] = cmd[ReadMemType].cmd;
    tx[0] = (tx[0] << 8);

    uint8_t spiDelay = IS25LP016D_Transmit(ReadMemType, tx);

    if(spiDelay > 0)
    {
        mibspiGetData(mibspiREG5, cmd[ReadManId].mibSPIGroup, tx);
        *memType = tx[1];
        res = TRUE;
    }

    return res;
}

/* read data */
bool IS25LP016D_ReadData(uint32_t startAddress, uint32_t byteSize, uint8 * destBuff)
{
    bool error = FALSE;

    if ((startAddress + byteSize) < IS25LP016D_BYTE_SIZE)
    {
        uint32_t address;
        uint16_t spiDelay;
        uint8_t* pt;
        uint8_t sessionDataCnt;
        uint16_t readCycle = byteSize/(IS25LP016D_SESSION_DATA_WORD_SIZE);
        uint8_t  byteRest = byteSize%IS25LP016D_SESSION_DATA_WORD_SIZE;
        int i,k;
        /* check if one more cycle is needed*/
        if(byteRest > 0u)
        {
            readCycle++;
        }

        IS25LP016D_internalBuff[0] = cmd[ReadDataNormal].cmd;
        IS25LP016D_internalBuff[0] = (IS25LP016D_internalBuff[0] << 8);

        for (i = 0; i < readCycle; i++)
        {
            address = startAddress + i*IS25LP016D_SESSION_DATA_WORD_SIZE;
            IS25LP016D_internalBuff[0] |= ( (address & 0x00FF0000) >> 16 );
            IS25LP016D_internalBuff[1] |= (address & 0x0000FFFF);
            spiDelay = IS25LP016D_TransmitLong(ReadDataNormal, IS25LP016D_internalBuff);

            if(spiDelay > 0)
            {
                mibspiGetData(mibspiREG5, cmd[ReadDataNormal].mibSPIGroup, IS25LP016D_internalBuff);

                pt = (uint8 *) &(IS25LP016D_internalBuff[2]);
                sessionDataCnt = IS25LP016D_SESSION_DATA_WORD_SIZE;

                /* calculate the byte number to fill */
                if (i == (readCycle - 1))
                {
                    if (byteRest > 0u)
                    {
                        /* last read cycle with rest */
                        sessionDataCnt = byteRest;
                    }
                }

                for(k = 0; k < sessionDataCnt; k++)
                {
                    *destBuff++ = *pt++;
                }
            }
            else
            {
                error = TRUE;
                break;
            }
        }
    }
    else
    {
        error = TRUE;
    }    

    return (!error);
}

/* write data */
bool IS25LP016D_WriteData(uint32_t startAddress, uint32_t byteSize, uint8 * sourceBuff)
{
    bool error = FALSE;

    if ((startAddress + byteSize) < IS25LP016D_BYTE_SIZE)
    {
        uint32_t address;
        uint16_t spiDelay;
        uint8_t sessionDataCnt;
        uint8_t * pt = (uint8_t*) IS25LP016D_internalBuff;
        uint16_t writeCycle = byteSize/(IS25LP016D_SESSION_DATA_WORD_SIZE);
        uint8_t  byteRest = byteSize%IS25LP016D_SESSION_DATA_WORD_SIZE;
        int i,k;
        
        /* check if one more cycle is needed*/
        if(byteRest > 0u)
        {
            writeCycle++;
        }

        IS25LP016D_internalBuff[0] = cmd[WriteData].cmd;
        IS25LP016D_internalBuff[0] = (IS25LP016D_internalBuff[0] << 8);

        for (i = 0; i < writeCycle; i++)
        {
            /* address */
            address = startAddress + i*IS25LP016D_SESSION_DATA_WORD_SIZE;
            IS25LP016D_internalBuff[0] |= ( (address & 0x00FF0000) >> 16 );
            IS25LP016D_internalBuff[1] |= (address & 0x0000FFFF);

            /* calculate the byte number to fill */
            sessionDataCnt = IS25LP016D_SESSION_DATA_WORD_SIZE;

            if (i == (writeCycle - 1))
            {
                if (byteRest > 0u)
                {
                    sessionDataCnt = byteRest;
                }
            }

            for(k = 0; k < sessionDataCnt; k++)
            {
                *(pt + k + 2) = *sourceBuff++;
            }

            /* padding the other bytes */
            for (k = sessionDataCnt; k < IS25LP016D_SESSION_DATA_WORD_SIZE; k++)
            {
                *(pt + k) = 0xFF;
            }

            spiDelay = IS25LP016D_TransmitLong(WriteData, IS25LP016D_internalBuff);
            if(spiDelay ==  0)
            {
                error = TRUE;
                break;
            }
        }
    }
    else
    {
        error = TRUE;
    }    

    return (!error);
}

/* write enable */
static bool IS25LP016D_WriteEnable (void)
{
    bool res = FALSE;
    uint8_t statusReg = 0;

    /* command write enable 0x06xx */
    uint16_t tx = cmd[WriteEnbl].cmd;

    uint8_t spiDelay = IS25LP016D_Transmit(WriteEnbl, &tx);

    /* check if write enable has been written */
    if (IS25LP016D_ReadStatusRegister(&statusReg))
    {
        if ( (statusReg & IS25LP016D_SR_WREN) == IS25LP016D_SR_WREN)
        {
            res = TRUE;
        }
    }

    return res;
}

/* Write status register */
static bool IS25LP016D_WriteStatusRegister (uint8*  status)
{
    bool res = FALSE;

    /* write enable before any erase */
    if (IS25LP016D_WriteEnable())
    {
        /* command */
        uint16_t tx = cmd[WriteStatusReg].cmd;
        tx = (tx << 8);
        tx |= (uint16) *status;

        uint8_t spiDelay = IS25LP016D_Transmit(WriteStatusReg, &tx);

        if(spiDelay > 0)
        {
            uint8 st = 0;

            if (IS25LP016D_ReadStatusRegister(&st))
            {
                if ( st == *status)
                {
                    res = TRUE;
                }
            }
        }
    }

    return res;
}

#if 0
bool IS25LP016D_WriteBlock (uint32 blockAddress)
{
    bool res = FALSE;

    /* write enable before any erase */
    if (IS25LP016D_WriteEnable())
    {
        /* command */
        IS25LP016D_internalBuff[0] = cmd[WriteData].cmd;
        IS25LP016D_internalBuff[0] = (IS25LP016D_internalBuff[0] << 8);

        /* address */
        IS25LP016D_internalBuff[0] |= ( (blockAddress & 0x00FF0000) >> 16 );
        IS25LP016D_internalBuff[1] |= (blockAddress & 0x0000FFFF);

        IS25LP016D_internalBuff[2] = 0x09;
        IS25LP016D_internalBuff[3] = 0x08;
        IS25LP016D_internalBuff[4] = 0x07;
        IS25LP016D_internalBuff[5] = 0x06;
        IS25LP016D_internalBuff[6] = 0x05;

        /* Read */
        uint8_t spiDelay = IS25LP016D_Transmit(WriteData, IS25LP016D_internalBuff);

        if(spiDelay > 0)
        {
            res = TRUE;
        }
    }

    return res;
}
#endif
/* erase block */
static bool IS25LP016D_EraseBlock (uint32 blockAddress, IS25LP016D_Cmd_E eraseMode)
{
    bool res = FALSE;

    if ( (eraseMode == EraseBlock64k) || (eraseMode == EraseBlock32k) || (eraseMode == EraseBlock4k) )
    {
        /* write enable before any erase */
        if (IS25LP016D_WriteEnable())
        {
            uint16_t tx[2] = {0};

            /* command */
            tx[0] = cmd[eraseMode].cmd;
            tx[0] = (tx[0] << 8);

            /* address */
            tx[0] |= ( (blockAddress & 0x00FF0000) >> 16 );
            tx[1] |= (blockAddress & 0x0000FFFF);

            /* Erase */
            uint8_t spiDelay = IS25LP016D_Transmit(eraseMode, tx);

            if(spiDelay > 0)
            {
                uint8_t statusReg = 0xFF;
                uint16_t EraseDelay = 0xFFFF;

                /* wait end of erase */
                while ( ((statusReg & IS25LP016D_SR_WIP) == IS25LP016D_SR_WIP) &&  (EraseDelay > 0u)  )
                {
                    IS25LP016D_ReadStatusRegister(&statusReg);
                    EraseDelay--;
                }

                if (EraseDelay > 0)
                {
                    res = TRUE;                     /* erase terminated */
                }
            }
        }

    }

    return res;
}

/* erase 64k block */
bool IS25LP016D_EraseBlock64k (uint32 blockAddress)
{
    return IS25LP016D_EraseBlock(blockAddress, EraseBlock64k);
}

/* erase 32k block */
bool IS25LP016D_EraseBlock32k (uint32 blockAddress)
{
    return IS25LP016D_EraseBlock(blockAddress, EraseBlock32k);
}

/* erase 4k block */
bool IS25LP016D_EraseBlock4k (uint32 blockAddress)
{
    return IS25LP016D_EraseBlock(blockAddress, EraseBlock4k);
}

/* erase entire memory */
bool IS25LP016D_EraseChip (void)
{
    bool res = FALSE;

    /* write enable before any erase */
    if (IS25LP016D_WriteEnable())
    {
        uint16_t tx = cmd[EraseChip].cmd;

        uint8_t spiDelay = IS25LP016D_Transmit(EraseChip, &tx);

        if(spiDelay > 0)
        {
            uint8_t statusReg = 0xFF;
            uint32_t EraseEntireMemDelay = 0x00FFFFFF;

            /* wait end of erase */
            while ( ((statusReg & IS25LP016D_SR_WIP) == IS25LP016D_SR_WIP) &&  (EraseEntireMemDelay > 0u)  )
            {
                IS25LP016D_ReadStatusRegister(&statusReg);
                EraseEntireMemDelay--;
            }

            if (EraseEntireMemDelay > 0)
            {
                res = TRUE;                     /*  erase terminated */
            }
        }
    }

    return res;
}

/* write enable */
static bool IS25LP016D_ClearExtendedStatusRegister (void)
{
    bool res = FALSE;

    uint16_t tx = cmd[ClearExtReadReg].cmd;
    uint8_t spiDelay = IS25LP016D_Transmit(ClearExtReadReg, &tx);

    /* check if write enable has been written */
    if (spiDelay > 0)
    {
        res = TRUE;
    }

    return res;
}

/* Exit power down mode */
static bool IS25LP016D_SwReset (void)
{
    bool res = FALSE;

    /* Sw Reset enbl */
    uint16_t tx = cmd[SwResetEnbl].cmd;
    uint8_t spiDelay = IS25LP016D_Transmit(SwResetEnbl, &tx);

    /* Sw reset */
    if(spiDelay > 0)
    {
        tx = cmd[SwReset].cmd;
        spiDelay = IS25LP016D_Transmit(SwReset, &tx);

        if (spiDelay > 0)
        {
            res = TRUE;
        }
    }

    return res;
}

/* Enter power down mode */
static bool IS25LP016D_EnterPowerDown (void)
{
    bool res = FALSE;
    uint16_t tx = cmd[EnterPwDownMode].cmd;
    uint8_t spiDelay = IS25LP016D_Transmit(EnterPwDownMode, &tx);

    if (spiDelay > 0)
    {
        res = TRUE;
    }

    return res;
}

/* Exit power down mode */
static bool IS25LP016D_ReleasePowerDown (void)
{
    bool res = FALSE;
    uint16_t tx = cmd[ReleasePwDownMode].cmd;
    uint8_t spiDelay = IS25LP016D_Transmit(ReleasePwDownMode, &tx);

    if (spiDelay > 0)
    {
        res = TRUE;
    }

    return res;
}

/* Get the address of 64k block */
uint32_t IS25LP016D_GetBlock64kAddress (uint8_t id)
{
    uint32_t ret = 0xFFFFFFFF;

    if (id < 32)
    {
        ret = IS25LP016D_MAP_START_ADDRESS +  (IS25LP016D_BLOCK64k_BYTE_SIZE*id);
    }

    return ret;
}

/* Get the address of 32k block */
uint32_t IS25LP016D_GetBlock32kAddress (uint8_t id)
{
    uint32_t ret = 0xFFFFFFFF;

    if (id < 64)
    {
        ret = IS25LP016D_MAP_START_ADDRESS +  (IS25LP016D_BLOCK32k_BYTE_SIZE*id);
    }

    return ret;
}

/* Get the address of 4k sector */
uint32_t IS25LP016D_GetSector4kAddress (uint16_t id)
{
    uint32_t ret = 0xFFFFFFFF;

    if (id < 512)
    {
        ret = IS25LP016D_MAP_START_ADDRESS +  (IS25LP016D_BLOCK4k_BYTE_SIZE*id);
    }

    return ret;
}

bool IS25LP016D_TEST_ID (void)
{
    bool error = FALSE;
    uint8_t id = 0;
    uint16_t memType = 0;
    uint8_t statusReg = 0;

    /*< IDS >*/
    if (IS25LP016D_ReadDevID(&id))
    {
        if (id != IS25LP016D_DEV_ID)
        {
            error = TRUE;
        }
    }
    else
    {
        error = TRUE;
    }


    if (IS25LP016D_ReadManID(&id))
    {
        if (id != IS25LP016D_MANUFACTURER)
        {
            error = TRUE;
        }
    }
    else
    {
        error = TRUE;
    }

    if (IS25LP016D_ReadMemType(&memType))
    {
        if ( memType != IS25LP016D_MEM_TYPE_CAPACITY)
        {
            error = TRUE;
        }
    }
    else
    {
        error = TRUE;
    }

    if (IS25LP016D_ReadStatusRegister(&statusReg))
     {
         if ( (statusReg != 0) && (statusReg != 2) )
         {
             error = TRUE;
         }
     }
     else
     {
         error = TRUE;
     }


    /* write status register */
    uint8_t statusRegister =  0x02;

    if (IS25LP016D_WriteStatusRegister(&statusRegister))
    {

    }
    else
    {
        error = TRUE;
    }


    /* Check if Write enbl works */
    if (IS25LP016D_WriteEnable())
    {
        if (IS25LP016D_ReadStatusRegister(&statusReg))
        {
            if ( (statusReg & IS25LP016D_SR_WREN) != IS25LP016D_SR_WREN )
            {
                error = TRUE;
            }
        }
        else
        {
            error = TRUE;
        }
    }
    else
    {
        error = TRUE;
    }

    /* get some address */

    return error;
}

uint8_t rxProva[200] = {0};
uint8_t txProva[200] = {0};

bool IS25LP016D_TEST_ERASE_WRITE_READ (void)
{
    bool error = FALSE;
    static bool Done = FALSE;

    if (!Done)
    {
        Done = TRUE;

        if (!IS25LP016D_ReadData(0x00000000, 200, rxProva))
        {
            error = TRUE;
        }

        if (!IS25LP016D_EraseBlock32k(0x00000000))
        {
            error = TRUE;
        }

        txProva[0] = 1;
        txProva[1] = 2;
        txProva[2] = 3;
        txProva[197] = 6;
        txProva[198] = 7;
        txProva[199] = 8;


        if (!IS25LP016D_WriteData(0x00000000, 200, txProva))
        {
            error = TRUE;
        }

    }

    return error;
}


