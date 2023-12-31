/*
 * FtpServer esp8266 and esp32 with LittleFS
 *
 * AUTHOR:  Renzo Mischianti
 *
 * https://www.mischianti.org/2020/02/08/ftp-server-on-esp8266-and-esp32
 *
 */

#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>

#include <SimpleFTPServer.h>

const char *FTP_user = "admin";
const char *FTP_password = "admin";

FtpServer FTP; // set #define FTP_DEBUG in ESP8266FtpServer.h to see ftp verbose on serial

void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace)
{
    switch (ftpOperation)
    {
    case FTP_CONNECT:
        Serial.println(F("FTP: Connected!"));
        break;
    case FTP_DISCONNECT:
        Serial.println(F("FTP: Disconnected!"));
        break;
    case FTP_FREE_SPACE_CHANGE:
        Serial.printf("FTP: Free space change, free %u of %u!\n", freeSpace, totalSpace);
        break;
    default:
        break;
    }
};
void _transferCallback(FtpTransferOperation ftpOperation, const char *name, unsigned int transferredSize)
{
    switch (ftpOperation)
    {
    case FTP_UPLOAD_START:
        Serial.println(F("FTP: Upload start!"));
        break;
    case FTP_UPLOAD:
        Serial.printf("FTP: Upload of file %s byte %u\n", name, transferredSize);
        break;
    case FTP_TRANSFER_STOP:
        Serial.println(F("FTP: Finish transfer!"));
        break;
    case FTP_TRANSFER_ERROR:
        Serial.println(F("FTP: Transfer error!"));
        break;
    default:
        break;
    }

    /* FTP_UPLOAD_START = 0,
     * FTP_UPLOAD = 1,
     *
     * FTP_DOWNLOAD_START = 2,
     * FTP_DOWNLOAD = 3,
     *
     * FTP_TRANSFER_STOP = 4,
     * FTP_DOWNLOAD_STOP = 4,
     * FTP_UPLOAD_STOP = 4,
     *
     * FTP_TRANSFER_ERROR = 5,
     * FTP_DOWNLOAD_ERROR = 5,
     * FTP_UPLOAD_ERROR = 5
     */
};

void FTP_setup()
{
    LittleFS.begin();
    FTP.setCallback(_callback);
    FTP.setTransferCallback(_transferCallback);
    FTP.begin(FTP_user, FTP_password, "Erasinator FTP Server"); // username, password for ftp.   (default 21, 50009 for PASV)
}
void FTP_loop(void)
{
    FTP.handleFTP(); // make sure in loop you call handleFTP()!!
}