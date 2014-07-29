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

#ifndef SD_FILE_SYSTEM_H
#define SD_FILE_SYSTEM_H

#include "mbed.h"
#include "FATFileSystem.h"
#include <stdint.h>

/** SDFileSystem class.
 *  Used for creating a virtual file system for accessing SD/MMC cards via SPI.
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "SDFileSystem.h"
 *
 * //Create an SDFileSystem object
 * SDFileSystem sd(p5, p6, p7, p19, p20, "sd")
 *
 * int main()
 * {
 *     //Perform a write test
 *     printf("\nWriting to SD card...");
 *     FILE *fp = fopen("/sd/sdtest.txt", "w");
 *     if (fp != NULL) {
 *         fprintf(fp, "We're writing to an SD card!");
 *         fclose(fp);
 *         printf("success!\n");
 *     } else {
 *         printf("failed!\n");
 *     }
 *
 *     //Perform a read test
 *     printf("Reading from SD card...");
 *     fp = fopen("/sd/sdtest.txt", "r");
 *     if (fp != NULL) {
 *         char c = fgetc(fp);
 *         if (c == 'W')
 *             printf("success!\n");
 *         else
 *             printf("incorrect char (%c)!\n", c);
 *         fclose(fp);
 *     } else {
 *         printf("failed!\n");
 *     }
 * }
 * @endcode
 */
class SDFileSystem : public FATFileSystem
{
public:
    /** Represents the different SD/MMC card types
     */
    enum CardType {
        CARD_NONE,      /**< No card is present */
        CARD_MMC,       /**< MMC card */
        CARD_SD,        /**< Standard capacity SD card */
        CARD_SDHC,      /**< High capacity SD card */
        CARD_UNKNOWN    /**< Unknown or unsupported card */
    };

    /** Create a virtual file system for accessing SD/MMC cards via SPI
     *
     * @param mosi The SPI data out pin.
     * @param miso The SPI data in pin.
     * @param sclk The SPI clock pin.
     * @param cs The SPI chip select pin.
     * @param cd The active-high card detect pin.
     * @param name The name used to access the virtual filesystem.
     * @param hz The SPI bus frequency (defaults to 1MHz).
     */
    SDFileSystem(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName cd, const char* name, int hz = 1000000);

    /** Get the detected SD/MMC card type
     *
     * @returns The detected card type as a CardType enum.
     */
    SDFileSystem::CardType card_type();

    virtual int disk_initialize();
    virtual int disk_status();
    virtual int disk_read(uint8_t* buffer, uint64_t sector);
    virtual int disk_write(const uint8_t* buffer, uint64_t sector);
    virtual int disk_sync();
    virtual uint64_t disk_sectors();

private:
    //Commands
    enum Command {
        CMD0 = 0,       /**< GO_IDLE_STATE */
        CMD1 = 1,       /**< SEND_OP_COND */
        ACMD41 = 41,    /**< APP_SEND_OP_COND */
        CMD8 = 8,       /**< SEND_IF_COND */
        CMD9 = 9,       /**< SEND_CSD */
        CMD16 = 16,     /**< SET_BLOCKLEN */
        CMD17 = 17,     /**< READ_SINGLE_BLOCK */
        CMD24 = 24,     /**< WRITE_BLOCK */
        CMD55 = 55,     /**< APP_CMD */
        CMD58 = 58,     /**< READ_OCR */
        CMD59 = 59      /**< CRC_ON_OFF */
    };

    //Member variables
    SPI m_SPI;
    DigitalOut m_CS;
    InterruptIn m_CD;
    int m_SpiFreq;
    int m_Status;
    SDFileSystem::CardType m_CardType;

    //Internal methods
    void checkSocket();
    bool waitReady(int timeout);
    bool select();
    void deselect();
    char writeCommand(char cmd, unsigned int arg);
    unsigned int readReturn();
    bool readData(char* buffer, int length);
};

#endif
