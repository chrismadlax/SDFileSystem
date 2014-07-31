/* SD/MMC File System Library
 * Copyright (c) 2014 Neil Thiessen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SDFileSystem.h"
#include "diskio.h"
#include "CRC7.h"
#include "CRC16.h"

SDFileSystem::SDFileSystem(PinName mosi, PinName miso, PinName sclk, PinName cs, const char* name, PinName cd, SwitchType cdtype, int hz) : FATFileSystem(name), m_SPI(mosi, miso, sclk), m_CS(cs, 1), m_CD(cd), m_CD_ASSERT((int)cdtype)
{
    //Initialize the member variables
    m_SpiFreq = hz;
    m_Status = STA_NOINIT;
    m_CardType = CARD_NONE;

    //Configure the SPI bus
    m_SPI.format(8, 0);

    //Configure the card detect pin
    m_CD.mode(PullUp);
    if (cdtype == SWITCH_NO)
        m_CD.rise(this, &SDFileSystem::checkSocket);
    else
        m_CD.fall(this, &SDFileSystem::checkSocket);
}

SDFileSystem::CardType SDFileSystem::card_type()
{
    //Check the card socket
    checkSocket();

    //If a card is present but not initialized, initialize it
    if (!(m_Status & STA_NODISK) && (m_Status & STA_NOINIT))
        disk_initialize();

    //Return the card type
    return m_CardType;
}

int SDFileSystem::disk_initialize()
{
    char resp;

    //Make sure there's a card in the socket before proceeding
    checkSocket();
    if (m_Status & STA_NODISK)
        return m_Status;

    //Make sure we're not already initialized before proceeding
    if (!(m_Status & STA_NOINIT))
        return m_Status;

    //Set the SPI frequency to 100kHz for initialization
    m_SPI.frequency(100000);

    //Send 80 dummy clocks with /CS and DI held high
    m_CS = 1;
    for (int i = 0; i < 10; i++)
        m_SPI.write(0xFF);

    //Write CMD0(0x00000000) to reset the card
    resp = writeCommand(CMD0, 0x00000000);
    if (resp != 0x01) {
        //Initialization failed
        m_CardType = CARD_UNKNOWN;
        return m_Status;
    }

    //Write CMD8(0x000001AA) to see if this is an SDCv2 card
    resp = writeCommand(CMD8, 0x000001AA);
    if (resp == 0x01) {
        //This is an SDCv2 card, get the 32-bit return value and verify the voltage range/check pattern
        if ((readReturn() & 0xFFF) != 0x1AA) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Send CMD58(0x00000000) to read the OCR, and verify that the card supports 3.2-3.3V
        resp = writeCommand(CMD58, 0x00000000);
        if (resp != 0x01 || !(readReturn() & (1 << 20))) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Send ACMD41(0x40100000) repeatedly for up to 1 second to initialize the card
        for (int i = 0; i < 1000; i++) {
            resp = writeCommand(ACMD41, 0x40100000);
            if (resp != 0x01)
                break;
            wait_ms(1);
        }

        //Check if the card initialized
        if (resp != 0x00) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Send CMD58(0x00000000) to read the OCR
        resp = writeCommand(CMD58, 0x00000000);
        if (resp == 0x00) {
            //Check the CCS bit to determine if this is a high capacity card
            if (readReturn() & (1 << 30))
                m_CardType = CARD_SDHC;
            else
                m_CardType = CARD_SD;
        } else {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }
    } else {
        //Didn't respond or illegal command, this is either an SDCv1 or MMC card
        //Send CMD58(0x00000000) to read the OCR, and verify that the card supports 3.2-3.3V
        resp = writeCommand(CMD58, 0x00000000);
        if (resp != 0x01 || !(readReturn() & (1 << 20))) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Try to initialize the card using ACMD41(0x00100000) for 1 second
        for (int i = 0; i < 1000; i++) {
            resp = writeCommand(ACMD41, 0x00100000);
            if (resp != 0x01)
                break;
            wait_ms(1);
        }

        //Check if the card initialized
        if (resp == 0x00) {
            //This is an SDCv1 standard capacity card
            m_CardType = CARD_SD;
        } else {
            //Try to initialize the card using CMD1(0x00100000) for 1 second
            for (int i = 0; i < 1000; i++) {
                resp = writeCommand(CMD1, 0x00100000);
                if (resp != 0x01)
                    break;
                wait_ms(1);
            }

            //Check if the card initialized
            if (resp == 0x00) {
                //This is an MMCv3 card
                m_CardType = CARD_MMC;
            } else {
                //Initialization failed
                m_CardType = CARD_UNKNOWN;
                return m_Status;
            }
        }
    }

    //Send CMD59(0x00000001) to re-enable CRC
    resp = writeCommand(CMD59, 0x00000001);
    if (resp != 0x00) {
        //Initialization failed
        m_CardType = CARD_UNKNOWN;
        return m_Status;
    }

    //Send CMD16(0x00000200) to force the block size to 512B if necessary
    if (m_CardType != CARD_SDHC) {
        resp = writeCommand(CMD16, 0x00000200);
        if (resp != 0x00) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }
    }

    //Send ACMD42(0x00000000) to disconnect the internal pull-up resistor on /CS if necessary
    if (m_CardType != CARD_MMC) {
        resp = writeCommand(ACMD42, 0x00000000);
        if (resp != 0x00) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }
    }

    //The card is now initialized
    m_Status &= ~STA_NOINIT;

    //Increase the SPI frequency to full speed (limited to 20MHz for MMC, or 25MHz for SDC)
    if (m_CardType == CARD_MMC && m_SpiFreq > 20000000)
        m_SPI.frequency(20000000);
    else if (m_SpiFreq > 25000000)
        m_SPI.frequency(25000000);
    else
        m_SPI.frequency(m_SpiFreq);

    //Return the device status
    return m_Status;
}

int SDFileSystem::disk_status()
{
    //Check if there's a card in the socket
    checkSocket();

    //Return the device status
    return m_Status;
}

int SDFileSystem::disk_read(uint8_t* buffer, uint64_t sector)
{
    //Make sure the device is initialized before proceeding
    if (m_Status & STA_NOINIT)
        return RES_NOTRDY;

    //Convert from LBA to a byte address for standard capacity cards
    if (m_CardType != CARD_SDHC)
        sector *= 512;

    //Try to read the block up to 3 times
    for (int i = 0; i < 3; i++) {
        //Send CMD17(sector) to read a single block
        char resp = writeCommand(CMD17, sector);
        if (resp == 0x00) {
            //Try to read the sector, and return if successful
            if (readData((char*)buffer, 512))
                return RES_OK;
        } else {
            //The command failed
            return RES_ERROR;
        }
    }

    //The read operation failed 3 times (CRC most likely)
    return RES_ERROR;
}

int SDFileSystem::disk_write(const uint8_t* buffer, uint64_t sector)
{
    //Make sure the device is initialized before proceeding
    if (m_Status & STA_NOINIT)
        return RES_NOTRDY;

    //Make sure the device isn't write protected before proceeding
    if (m_Status & STA_PROTECT)
        return RES_WRPRT;

    //Convert from LBA to a byte address for older cards
    if (m_CardType != CARD_SDHC)
        sector *= 512;

    //Try to write the block up to 3 times
    for (int i = 0; i < 3; i++) {
        //Send CMD24(sector) to write a single block
        if (writeCommand(CMD24, sector) == 0x00) {
            //Wait for up to 500ms for the card to become ready
            if (!waitReady(500)) {
                //We timed out, deselect and loop again
                deselect();
                continue;
            }

            //Send the write data token
            m_SPI.write(0xFE);

            //Write the data block from the buffer
            for (int b = 0; b < 512; b++)
                m_SPI.write(buffer[b]);

            //Calculate the CRC16 checksum for the data block and send it
            unsigned short crc = CRC16((char*)buffer, 512);
            m_SPI.write(crc >> 8);
            m_SPI.write(crc);

            //Receive the data response, and deselect the card
            char resp = m_SPI.write(0xFF) & 0x1F;
            deselect();

            //Check the response
            if (resp == 0x05)
                return RES_OK;
            else if (resp == 0x0D)
                return RES_ERROR;
        } else {
            //The command failed
            return RES_ERROR;
        }
    }

    //The operation either timed out 3 times, failed the CRC check 3 times, or experienced a write error
    return RES_ERROR;
}

int SDFileSystem::disk_sync()
{
    //Select the card so we're forced to wait for the end of any internal write processes
    bool ret = select();
    deselect();

    //Return success/failure
    return (ret) ? RES_OK : RES_ERROR;
}

uint64_t SDFileSystem::disk_sectors()
{
    //Make sure the device is initialized before proceeding
    if (m_Status & STA_NOINIT)
        return 0;

    //Try to read the CSD register up to 3 times
    for (int i = 0; i < 3; i++) {
        //Send CMD9(0x00000000) to read the CSD register
        if (writeCommand(CMD9, 0x00000000) == 0x00) {
            //Receive the 16B CSD data
            char csd[16];
            if (readData(csd, 16)) {
                //Calculate the sector count based on the card type
                if ((csd[0] >> 6) == 0x01) {
                    //Calculate the sector count a high capacity card
                    uint64_t sectors = (((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9]) + 1;
                    return sectors << 10;
                } else {
                    //Calculate the sector count standard capacity card
                    uint64_t sectors = (((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] & 0xC0) >> 6)) + 1;
                    sectors <<= ((((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7)) + 2);
                    sectors <<= (csd[5] & 0x0F);
                    return sectors >> 9;
                }
            }
        } else {
            //The command failed
            return 0;
        }
    }

    //The read operation failed 3 times (CRC most likely)
    return 0;
}

void SDFileSystem::checkSocket()
{
    //Check if a card is in the socket
    if (m_CD == m_CD_ASSERT) {
        //The socket is occupied, clear the STA_NODISK flag
        m_Status &= ~STA_NODISK;
    } else {
        //The socket is empty
        m_Status |= (STA_NODISK | STA_NOINIT);
        m_CardType = CARD_NONE;
    }
}

inline bool SDFileSystem::waitReady(int timeout)
{
    //Wait for the specified timeout for the card to become ready
    for (int i = 0; i < timeout; i++) {
        if (m_SPI.write(0xFF) == 0xFF)
            return true;
        wait_ms(1);
    }

    //We timed out
    return false;
}

inline bool SDFileSystem::select()
{
    //Pull /CS low
    m_CS = 0;

    //Send a dummy clock to enable DO
    m_SPI.write(0xFF);

    //Wait for up to 500ms for the card to become ready
    if (waitReady(500))
        return true;

    //We timed out, deselect and return false
    deselect();
    return false;
}

inline void SDFileSystem::deselect()
{
    //Pull /CS high
    m_CS = 1;

    //Send a dummy byte to release DO
    m_SPI.write(0xFF);
}

char SDFileSystem::writeCommand(char cmd, unsigned int arg)
{
    char resp;

    //Try to send the command up to 3 times
    for (int i = 0; i < 3; i++) {
        //Send CMD55(0x00000000) prior to an application specific command
        if (cmd == ACMD41 || cmd == ACMD42) {
            resp = writeCommand(CMD55, 0x00000000);
            if (resp > 0x01)
                return resp;
        }

        //Select the card, and wait for ready
        if (!select())
            return 0xFF;

        //Prepare the command packet
        char cmdPacket[6];
        cmdPacket[0] = cmd;
        cmdPacket[1] = arg >> 24;
        cmdPacket[2] = arg >> 16;
        cmdPacket[3] = arg >> 8;
        cmdPacket[4] = arg;
        cmdPacket[5] = (CRC7(cmdPacket, 5) << 1) | 0x01;

        //Send the command packet
        for (int b = 0; b < 6; b++)
            m_SPI.write(cmdPacket[b]);

        //Allow up to 10 bytes of delay for the command response
        for (int b = 0; b < 10; b++) {
            resp = m_SPI.write(0xFF);
            if (!(resp & 0x80))
                break;
        }

        //Deselect the card on errors, or if the transaction is finished
        if (resp > 0x01 || !(cmd == CMD8 || cmd == CMD9 || cmd == CMD17 || cmd == CMD24 || cmd == CMD55 || cmd == CMD58))
            deselect();

        //Return the response unless there were CRC errors
        if (resp == 0xFF || !(resp & (1 << 3)))
            return resp;
    }

    //The command failed 3 times due to CRC errors
    return 0xFF;
}

unsigned int SDFileSystem::readReturn()
{
    unsigned int ret;

    //Read the 32-bit response value
    ret = (m_SPI.write(0xFF) << 24);
    ret |= (m_SPI.write(0xFF) << 16);
    ret |= (m_SPI.write(0xFF) << 8);
    ret |= m_SPI.write(0xFF);

    //Deselect the card
    deselect();

    //Return the response value
    return ret;
}

bool SDFileSystem::readData(char* buffer, int length)
{
    char token;

    //Wait for up to 200ms for the DataStart token to arrive
    for (int i = 0; i < 200; i++) {
        token = m_SPI.write(0xFF);
        if (token != 0xFF)
            break;
        wait_ms(1);
    }

    //Make sure the token is valid
    if (token != 0xFE)
        return false;

    //Read the data into the buffer
    for (int i = 0; i < length; i++)
        buffer[i] = m_SPI.write(0xFF);

    //Read the CRC16 checksum for the data block, and deselect the card
    unsigned short crc = (m_SPI.write(0xFF) << 8);
    crc |= m_SPI.write(0xFF);
    deselect();

    //Indicate whether the CRC16 checksum was valid or not
    if (crc == CRC16(buffer, length))
        return true;
    else
        return false;
}
