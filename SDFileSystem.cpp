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

SDFileSystem::SDFileSystem(PinName mosi, PinName miso, PinName sclk, PinName cs, const char* name, PinName cd, SwitchType cdtype, int hz) : FATFileSystem(name), m_Spi(mosi, miso, sclk), m_Cs(cs, 1), m_Cd(cd), m_CD_ASSERT((int)cdtype), m_FREQ(hz)
{
    //Initialize the member variables
    m_CardType = CARD_NONE;
    m_Crc = true;
    m_LargeFrames = false;
    m_Status = STA_NOINIT;

    //Configure the SPI bus
    m_Spi.format(8, 0);

    //Configure the card detect pin
    m_Cd.mode(PullUp);
    if (cdtype == SWITCH_NO)
        m_Cd.rise(this, &SDFileSystem::checkSocket);
    else
        m_Cd.fall(this, &SDFileSystem::checkSocket);
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

bool SDFileSystem::crc()
{
    //Return whether or not CRC is enabled
    return m_Crc;
}

void SDFileSystem::crc(bool enabled)
{
    //Check the card socket
    checkSocket();

    //Just update the member variable if the card isn't initialized
    if (m_Status & STA_NOINIT) {
        m_Crc = enabled;
        return;
    }

    //Enable or disable CRC
    if (enabled && !m_Crc) {
        //Send CMD59(0x00000001) to enable CRC
        m_Crc = true;
        commandTransaction(CMD59, 0x00000001);
    } else if (!enabled && m_Crc) {
        //Send CMD59(0x00000000) to disable CRC
        commandTransaction(CMD59, 0x00000000);
        m_Crc = false;
    }
}

bool SDFileSystem::large_frames()
{
    //Return whether or not 16-bit frames are enabled
    return m_LargeFrames;
}

void SDFileSystem::large_frames(bool enabled)
{
    //Set whether or not 16-bit frames are enabled
    m_LargeFrames = enabled;
}

int SDFileSystem::unmount()
{
    //Unmount the filesystem
    FATFileSystem::unmount();

    //Change the status to not initialized, and the card type to none
    m_Status |= STA_NOINIT;
    m_CardType = CARD_NONE;

    //Always succeeds
    return 0;
}

int SDFileSystem::disk_initialize()
{
    char token;
    unsigned int resp;

    //Make sure there's a card in the socket before proceeding
    checkSocket();
    if (m_Status & STA_NODISK)
        return m_Status;

    //Make sure we're not already initialized before proceeding
    if (!(m_Status & STA_NOINIT))
        return m_Status;

    //Set the SPI frequency to 400kHz for initialization
    m_Spi.frequency(400000);

    //Send 80 dummy clocks with /CS deasserted and DI held high
    m_Cs = 1;
    for (int i = 0; i < 10; i++)
        m_Spi.write(0xFF);

    //Write CMD0(0x00000000) to reset the card
    if (commandTransaction(CMD0, 0x00000000) != 0x01) {
        //Initialization failed
        m_CardType = CARD_UNKNOWN;
        return m_Status;
    }

    //Send CMD59(0x00000001) to enable CRC if necessary
    if (m_Crc) {
        if (commandTransaction(CMD59, 0x00000001) != 0x01) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }
    }

    //Write CMD8(0x000001AA) to see if this is an SDCv2 card
    if (commandTransaction(CMD8, 0x000001AA, &resp) == 0x01) {
        //This is an SDCv2 card, get the 32-bit return value and verify the voltage range/check pattern
        if ((resp & 0xFFF) != 0x1AA) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Send CMD58(0x00000000) to read the OCR, and verify that the card supports 3.2-3.3V
        if (commandTransaction(CMD58, 0x00000000, &resp) != 0x01 || !(resp & (1 << 20))) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Send ACMD41(0x40100000) repeatedly for up to 1 second to initialize the card
        for (int i = 0; i < 1000; i++) {
            token = commandTransaction(ACMD41, 0x40100000);
            if (token != 0x01)
                break;
            wait_ms(1);
        }

        //Check if the card initialized
        if (token != 0x00) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Send CMD58(0x00000000) to read the OCR
        if (commandTransaction(CMD58, 0x00000000, &resp) == 0x00) {
            //Check the CCS bit to determine if this is a high capacity card
            if (resp & (1 << 30))
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
        if (commandTransaction(CMD58, 0x00000000, &resp) != 0x01 || !(resp & (1 << 20))) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }

        //Try to initialize the card using ACMD41(0x00100000) for 1 second
        for (int i = 0; i < 1000; i++) {
            token = commandTransaction(ACMD41, 0x00100000);
            if (token != 0x01)
                break;
            wait_ms(1);
        }

        //Check if the card initialized
        if (token == 0x00) {
            //This is an SDCv1 standard capacity card
            m_CardType = CARD_SD;
        } else {
            //Try to initialize the card using CMD1(0x00100000) for 1 second
            for (int i = 0; i < 1000; i++) {
                token = commandTransaction(CMD1, 0x00100000);
                if (token != 0x01)
                    break;
                wait_ms(1);
            }

            //Check if the card initialized
            if (token == 0x00) {
                //This is an MMCv3 card
                m_CardType = CARD_MMC;
            } else {
                //Initialization failed
                m_CardType = CARD_UNKNOWN;
                return m_Status;
            }
        }
    }

    //Send CMD16(0x00000200) to force the block size to 512B if necessary
    if (m_CardType != CARD_SDHC) {
        if (commandTransaction(CMD16, 0x00000200) != 0x00) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }
    }

    //Send ACMD42(0x00000000) to disconnect the internal pull-up resistor on pin 1 if necessary
    if (m_CardType != CARD_MMC) {
        if (commandTransaction(ACMD42, 0x00000000) != 0x00) {
            //Initialization failed
            m_CardType = CARD_UNKNOWN;
            return m_Status;
        }
    }

    //The card is now initialized
    m_Status &= ~STA_NOINIT;

    //Increase the SPI frequency to full speed (limited to 20MHz for MMC, or 25MHz for SDC)
    if (m_CardType == CARD_MMC && m_FREQ > 20000000)
        m_Spi.frequency(20000000);
    else if (m_FREQ > 25000000)
        m_Spi.frequency(25000000);
    else
        m_Spi.frequency(m_FREQ);

    //Return the disk status
    return m_Status;
}

int SDFileSystem::disk_status()
{
    //Check if there's a card in the socket
    checkSocket();

    //Return the disk status
    return m_Status;
}

int SDFileSystem::disk_read(uint8_t* buffer, uint64_t sector, uint8_t count)
{
    //Make sure the card is initialized before proceeding
    if (m_Status & STA_NOINIT)
        return RES_NOTRDY;

    //Read a single block, or multiple blocks
    if (count > 1) {
        if (readBlocks((char*)buffer, sector, count))
            return RES_OK;
    } else {
        if (readBlock((char*)buffer, sector))
            return RES_OK;
    }

    //The read operation failed
    return RES_ERROR;
}

int SDFileSystem::disk_write(const uint8_t* buffer, uint64_t sector, uint8_t count)
{
    //Make sure the card is initialized before proceeding
    if (m_Status & STA_NOINIT)
        return RES_NOTRDY;

    //Make sure the card isn't write protected before proceeding
    if (m_Status & STA_PROTECT)
        return RES_WRPRT;

    //Write a single block, or multiple blocks
    if (count > 1) {
        if(writeBlocks((const char*)buffer, sector, count))
            return RES_OK;
    } else {
        if(writeBlock((const char*)buffer, sector))
            return RES_OK;
    }

    //The write operation failed
    return RES_ERROR;
}

int SDFileSystem::disk_sync()
{
    //Select the card so we're forced to wait for the end of any internal write processes
    if (select()) {
        deselect();
        return RES_OK;
    } else {
        return RES_ERROR;
    }
}

uint64_t SDFileSystem::disk_sectors()
{
    //Make sure the card is initialized before proceeding
    if (m_Status & STA_NOINIT)
        return 0;

    //Try to read the CSD register up to 3 times
    for (int f = 0; f < 3; f++) {
        //Select the card, and wait for ready
        if (!select())
            break;

        //Send CMD9(0x00000000) to read the CSD register
        if (command(CMD9, 0x00000000) == 0x00) {
            //Read the 16B CSD data block
            char csd[16];
            bool success = readData(csd, 16);
            deselect();
            if (success) {
                //Calculate the sector count based on the card type
                if ((csd[0] >> 6) == 0x01) {
                    //Calculate the sector count for a high capacity card
                    uint64_t size = (((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9]) + 1;
                    return size << 10;
                } else {
                    //Calculate the sector count for a standard capacity card
                    uint64_t size = (((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] & 0xC0) >> 6)) + 1;
                    size <<= ((((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7)) + 2);
                    size <<= (csd[5] & 0x0F);
                    return size >> 9;
                }
            }
        } else {
            //The command failed, get out
            break;
        }
    }

    //The read operation failed 3 times
    deselect();
    return 0;
}

void SDFileSystem::checkSocket()
{
    //Check if a card is in the socket
    if (m_Cd == m_CD_ASSERT) {
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
        if (m_Spi.write(0xFF) == 0xFF)
            return true;
        wait_ms(1);
    }

    //We timed out
    return false;
}

inline bool SDFileSystem::select()
{
    //Assert /CS
    m_Cs = 0;

    //Send 8 dummy clocks with DI held high to enable DO
    m_Spi.write(0xFF);

    //Wait for up to 500ms for the card to become ready
    if (waitReady(500)) {
        return true;
    } else {
        //We timed out, deselect and return false
        deselect();
        return false;
    }
}

inline void SDFileSystem::deselect()
{
    //Deassert /CS
    m_Cs = 1;

    //Send 8 dummy clocks with DI held high to disable DO
    m_Spi.write(0xFF);
}

inline char SDFileSystem::commandTransaction(char cmd, unsigned int arg, unsigned int* resp)
{
    //Select the card, and wait for ready
    if (!select())
        return 0xFF;

    //Perform the command transaction
    char token = command(cmd, arg, resp);

    //Deselect the card, and return the R1 response token
    deselect();
    return token;
}

char SDFileSystem::command(char cmd, unsigned int arg, unsigned int* resp)
{
    char token;

    //Try to send the command up to 3 times
    for (int f = 0; f < 3; f++) {
        //Send CMD55(0x00000000) prior to an application specific command
        if (cmd == ACMD23 || cmd == ACMD41 || cmd == ACMD42) {
            token = command(CMD55, 0x00000000);
            if (token > 0x01)
                return token;

            //Some cards need a dummy byte between CMD55 and an ACMD
            m_Spi.write(0xFF);
        }

        //Prepare the command packet
        char cmdPacket[6];
        cmdPacket[0] = cmd;
        cmdPacket[1] = arg >> 24;
        cmdPacket[2] = arg >> 16;
        cmdPacket[3] = arg >> 8;
        cmdPacket[4] = arg;
        if (m_Crc || cmd == CMD0 || cmd == CMD8)
            cmdPacket[5] = (CRC7(cmdPacket, 5) << 1) | 0x01;
        else
            cmdPacket[5] = 0x01;

        //Send the command packet
        for (int i = 0; i < 6; i++)
            m_Spi.write(cmdPacket[i]);

        //Discard the stuff byte immediately following CMD12
        if (cmd == CMD12)
            m_Spi.write(0xFF);

        //Allow up to 8 bytes of delay for the R1 response token
        for (int i = 0; i < 9; i++) {
            token = m_Spi.write(0xFF);
            if (!(token & 0x80))
                break;
        }

        //Verify the R1 response token
        if (token == 0xFF) {
            //No data was received, get out early
            break;
        } else if (token & (1 << 3)) {
            //There was a CRC error, try again
            continue;
        } else if (token > 0x01) {
            //An error occured, get out early
            break;
        }

        //Handle R2 and R3/R7 response tokens
        if (cmd == CMD13 && resp != NULL) {
            //Read the R2 response value
            *resp = m_Spi.write(0xFF);
        } else if ((cmd == CMD8 || cmd == CMD58) && resp != NULL) {
            //Read the R3/R7 response value
            *resp = (m_Spi.write(0xFF) << 24);
            *resp |= (m_Spi.write(0xFF) << 16);
            *resp |= (m_Spi.write(0xFF) << 8);
            *resp |= m_Spi.write(0xFF);
        }

        //The command was successful
        break;
    }

    //Return the R1 response token
    return token;
}

bool SDFileSystem::readData(char* buffer, int length)
{
    char token;
    unsigned short crc;

    //Wait for up to 200ms for the start block token to arrive
    for (int i = 0; i < 200; i++) {
        token = m_Spi.write(0xFF);
        if (token != 0xFF)
            break;
        wait_ms(1);
    }

    //Make sure the token is valid
    if (token != 0xFE)
        return false;

    //Check if large frames are enabled or not
    if (m_LargeFrames) {
        //Switch to 16-bit frames for better performance
        m_Spi.format(16, 0);

        //Read the data block into the buffer
        unsigned short dataWord;
        for (int i = 0; i < length; i += 2) {
            dataWord = m_Spi.write(0xFFFF);
            buffer[i] = dataWord >> 8;
            buffer[i + 1] = dataWord;
        }

        //Read the CRC16 checksum for the data block
        crc = m_Spi.write(0xFFFF);

        //Switch back to 8-bit frames
        m_Spi.format(8, 0);
    } else {
        //Read the data into the buffer
        for (int i = 0; i < length; i++)
            buffer[i] = m_Spi.write(0xFF);

        //Read the CRC16 checksum for the data block
        crc = (m_Spi.write(0xFF) << 8);
        crc |= m_Spi.write(0xFF);
    }

    //Return the validity of the CRC16 checksum (if enabled)
    return (!m_Crc || crc == CRC16(buffer, length));
}

char SDFileSystem::writeData(const char* buffer, char token)
{
    //Calculate the CRC16 checksum for the data block (if enabled)
    unsigned short crc = (m_Crc) ? CRC16(buffer, 512) : 0xFFFF;

    //Wait for the card to become ready
    while (!m_Spi.write(0xFF));

    //Send the start block token
    m_Spi.write(token);

    //Check if large frames are enabled or not
    if (m_LargeFrames) {
        //Switch to 16-bit frames for better performance
        m_Spi.format(16, 0);

        //Write the data block from the buffer
        for (int i = 0; i < 512; i += 2)
            m_Spi.write((buffer[i] << 8) | buffer[i + 1]);

        //Send the CRC16 checksum for the data block
        m_Spi.write(crc);

        //Switch back to 8-bit frames
        m_Spi.format(8, 0);
    } else {
        //Write the data block from the buffer
        for (int i = 0; i < 512; i++)
            m_Spi.write(buffer[i]);

        //Send the CRC16 checksum for the data block
        m_Spi.write(crc >> 8);
        m_Spi.write(crc);
    }

    //Return the data response token
    return (m_Spi.write(0xFF) & 0x1F);
}

inline bool SDFileSystem::readBlock(char* buffer, unsigned long long lba)
{
    //Try to read the block up to 3 times
    for (int f = 0; f < 3; f++) {
        //Select the card, and wait for ready
        if (!select())
            break;

        //Send CMD17(block) to read a single block
        if (command(CMD17, (m_CardType == CARD_SDHC) ? lba : lba * 512) == 0x00) {
            //Try to read the block, and deselect the card
            bool success = readData(buffer, 512);
            deselect();

            //Return if successful
            if (success)
                return true;
        } else {
            //The command failed, get out
            break;
        }
    }

    //The single block read failed
    deselect();
    return false;
}

inline bool SDFileSystem::readBlocks(char* buffer, unsigned long long lba, int count)
{
    //Try to read each block up to 3 times
    for (int f = 0; f < 3;) {
        //Select the card, and wait for ready
        if (!select())
            break;

        //Send CMD18(block) to read multiple blocks
        if (command(CMD18, (m_CardType == CARD_SDHC) ? lba : lba * 512) == 0x00) {
            //Try to read all of the data blocks
            do {
                //Read the next block and break on errors
                if (!readData(buffer, 512)) {
                    f++;
                    break;
                }

                //Update the variables
                lba++;
                buffer += 512;
                f = 0;
            } while (--count);

            //Send CMD12(0x00000000) to stop the transmission
            if (command(CMD12, 0x00000000) != 0x00) {
                //The command failed, get out
                break;
            }

            //Only wait for CMD12 if the read was unsuccessful
            if (count)
                while (!m_Spi.write(0xFF));

            //Deselect the card
            deselect();

            //Return if successful
            if (count == 0)
                return true;
        } else {
            //The command failed, get out
            break;
        }
    }

    //The multiple block read failed
    deselect();
    return false;
}

inline bool SDFileSystem::writeBlock(const char* buffer, unsigned long long lba)
{
    //Try to write the block up to 3 times
    for (int f = 0; f < 3; f++) {
        //Select the card, and wait for ready
        if (!select())
            break;

        //Send CMD24(block) to write a single block
        if (command(CMD24, (m_CardType == CARD_SDHC) ? lba : lba * 512) == 0x00) {
            //Try to write the block, and deselect the card
            char token = writeData(buffer, 0xFE);
            deselect();

            //Check the data response token
            if (token == 0x0A) {
                //A CRC error occured, try again
                continue;
            } else if (token == 0x0C) {
                //A write error occured, get out
                break;
            }

            //Send CMD13(0x00000000) to verify that the programming was successful
            unsigned int resp;
            if (commandTransaction(CMD13, 0x00000000, &resp) != 0x00 || resp != 0x00) {
                //Some manner of unrecoverable write error occured during programming, get out
                break;
            }

            //The data was written successfully
            return true;
        } else {
            //The command failed, get out
            break;
        }
    }

    //The single block write failed
    deselect();
    return false;
}

inline bool SDFileSystem::writeBlocks(const char* buffer, unsigned long long lba, int count)
{
    char token;
    const char* currentBuffer = buffer;
    unsigned long long currentLba = lba;
    int currentCount = count;

    //Try to write each block up to 3 times
    for (int f = 0; f < 3;) {
        //If this is an SD card, send ACMD23(count) to set the number of blocks to pre-erase
        if (m_CardType != CARD_MMC) {
            if (commandTransaction(ACMD23, currentCount) != 0x00) {
                //The command failed, get out
                break;
            }
        }

        //Select the card, and wait for ready
        if (!select())
            break;

        //Send CMD25(block) to write multiple blocks
        if (command(CMD25, (m_CardType == CARD_SDHC) ? currentLba : currentLba * 512) == 0x00) {
            //Try to write all of the data blocks
            do {
                //Write the next block and break on errors
                token = writeData(currentBuffer, 0xFC);
                if (token != 0x05) {
                    f++;
                    break;
                }

                //Update the variables
                currentBuffer += 512;
                f = 0;
            } while (--currentCount);

            //Wait for the card to finish processing the last block
            while (!m_Spi.write(0xFF));

            //Finalize the transmission
            if (currentCount == 0) {
                //Send the stop tran token
                m_Spi.write(0xFD);

                //Wait for the programming to complete, and deselect the card
                while (!m_Spi.write(0xFF));
                deselect();

                //Send CMD13(0x00000000) to verify that the programming was successful
                unsigned int resp;
                if (commandTransaction(CMD13, 0x00000000, &resp) != 0x00 || resp != 0x00) {
                    //Some manner of unrecoverable write error occured during programming, get out
                    break;
                }

                //The data was written successfully
                return true;
            } else {
                //Send CMD12(0x00000000) to abort the transmission
                if (command(CMD12, 0x00000000) != 0x00) {
                    //The command failed, get out
                    break;
                }

                //Wait for CMD12 to complete, and deselect the card
                while (!m_Spi.write(0xFF));
                deselect();

                //Check the error token
                if (token == 0x0A) {
                    //Determine the number of well written blocks if possible
                    unsigned int writtenBlocks = 0;
                    if (m_CardType != CARD_MMC) {
                        //Send ACMD22(0x00000000) to get the number of well written blocks
                        if (commandTransaction(ACMD22, 0x00000000) == 0x00) {
                            //Read the data
                            char acmdData[4];
                            if (readData(acmdData, 4)) {
                                //Extract the number of well written blocks
                                writtenBlocks = acmdData[0] << 24;
                                writtenBlocks |= acmdData[1] << 16;
                                writtenBlocks |= acmdData[2] << 8;
                                writtenBlocks |= acmdData[3];
                            }
                        }
                    }

                    //Roll back the variables based on the number of well written blocks
                    currentBuffer = buffer + (writtenBlocks * 512);
                    currentLba = lba + writtenBlocks;
                    currentCount = count - writtenBlocks;

                    //Try again
                    continue;
                } else {
                    //A write error occured, get out
                    break;
                }
            }
        } else {
            //The command failed, get out
            break;
        }
    }

    //The multiple block write failed
    deselect();
    return false;
}
