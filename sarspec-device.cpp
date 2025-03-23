/**
 * Sarspec Linux usb device library
 * Project Name:
 * Design Name:
 * Linux Device Driver
 * working  with kernel 5.10
 *
 * Copyright  2024-present IPFN-Instituto Superior Tecnico, Portugal
 * Creation Date  2024-04-20
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include "sarspec-device.h"


#ifdef DEBUG
#endif

namespace sarspec_usb {
    const int DA_LEVELS = 65536;
    const int P = 3648;  //Toshiba 3648 pixels
    const int L = 3694;  //Toshiba 3694

    bool is_connected;

    SarspecResDevice::SarspecResDevice() : ftdi_dev(NULL)
    {
        if ((ftdi_dev = ftdi_new()) == 0)
        {
            std::cerr << "ftdi_new() failed" << std::endl;
            //std::cerr <<
            //<< std::endl;
        }
        ftdi_set_latency_timer(ftdi_dev, 1);
        ftdi_dev -> usb_read_timeout = 300000;
        ftdi_dev -> usb_write_timeout = 300000;

        std::cout << "usb type: " << ftdi_dev -> type << std::endl;

        //unsigned int chunkSize = 7385;
        //ftdi_read_data_set_chunksize(ftdi_dev, chunkSize);

        is_connected = false;
    }

    SarspecResDevice::~SarspecResDevice()
    {
        //disconnect();
    }

    /** Connects to device, sets initial dark and gain values to those stored in
      * EEPROM. Returns true on success, false on error.
      * \param[in] product product identification integer
      * \param[in] vendor vendor identification integer*/

    bool SarspecResDevice::connect(int vendor, int product)
    {
        int status;

        if (is_connected)
            return false;

        if ((status = ftdi_usb_open(ftdi_dev, vendor, product)) < 0)
        {
            //std::cerr <<
            //<< std::endl;
            std::cerr << "unable to open ftdi device: " << status
                << ftdi_get_error_string(ftdi_dev) << std::endl;
            ftdi_free(ftdi_dev);
            return false;
        }

        is_connected = true;

        unsigned char bufDark[] = {0x06, 0x00, 0x00};
        unsigned char bufGain[] = {0x08, 0x00, 0x00};

        ftdi_tcioflush(ftdi_dev);

        bufDark[1] = 9;
        bufDark[2] = 0;
        status = ftdi_write_data(ftdi_dev, bufDark, 3);
        if (status <= 0) {
            ftdi_free(ftdi_dev);
            return false;
        }

        bufGain[1] = 0x12;
        bufGain[2] = 0;
        status = ftdi_write_data(ftdi_dev, bufGain, 3);
        if (status <= 0) {
            ftdi_free(ftdi_dev);
            return false;
        }

        getCurrentEEPROMDarkGain();
        gain0 = darkGain[1];

        double d = setDark(darkGain[0]);
        double g = setGain(darkGain[1]);
        if (d == 0 || g == 0)
            return false;

        return true;
    }

    /** Disconnects device. Returns same as ftdi_usb_close() and -2 if device
    is not connected.*/
    int SarspecResDevice::disconnect()
    {

        if(!is_connected)
            return -2;

        int status;
        if ((status = ftdi_usb_close(ftdi_dev)) < 0)
        {
            std::cerr << "unable to close ftdi device: " << status
                << ftdi_get_error_string(ftdi_dev) << std::endl;
            //fprintf(stderr, "unable to close ftdi device: %d (%s)\n", status, ftdi_get_error_string(ftdi_dev));
            ftdi_free(ftdi_dev);
        }

        ftdi_free(ftdi_dev);

        is_connected = false;
        return status;
    }

    /** Sets LED ON/OFF. Returns same as ftdi_write_data().
      * \param[in] led ON/OFF value*/
    int SarspecResDevice::setLed(bool led)
    {

        if (!is_connected)
            return -666;

        int status;
        unsigned char bufLED[] = {0x18, 0x00};

        if (led) {
            bufLED[1] = 0x01;
        }

        status = ftdi_write_data(ftdi_dev, bufLED, 2);
        if (status < 0)
        {
            std::cerr << "write failed for: " << status
                << ftdi_get_error_string(ftdi_dev) << std::endl;
            //fprintf(stderr,"write failed for 0x%x, error %d (%s)\n",bufLED[0],status, ftdi_get_error_string(ftdi_dev));

            return status;
        }
        else
        {
            printf("ftdi LED write succeeded: %d\n",status);
        }

        return status;
    }

    /** Calculates values for wavelength using: c0 + c1n + c2n^2 + c3n^3 where n
      * is the pixel number.
      * Returns wavelength vector.
      * \param[in] coeffs c0, c1, c2, c3*/
    std::vector<double> SarspecResDevice::getXData(double c0, double c1, double c2, double c3)
    {
        xData.clear();
        for (int i = 0; i < P; i++)
        {
            xData.push_back(c0 + c1 * i + c2 * pow(double(i), 2) + c3 * pow(double(i), 3));
        }
        return xData;
    }

    /** Gets YData
      * Returns intensity vector.
      * \param[in] extTrigger whether to use external trigger.
      * \param[in] delay external trigger delay in ms. (not working)*/
    std::vector<double> SarspecResDevice::getYData(bool extTrigger, int delay)
    {
        if (!is_connected) {
            std::vector<double> v(3648, 0.0);
            return v;
        }

        int status;
        unsigned char bufStartScan[] = { 0x02, 0x00 };
        unsigned char bufStartScanExt[] = { 0x10, 0, 0, 0, 1 };
        unsigned char incomingBuf[8192];
        uint32_t bytesRead;
        yData.clear();

        if (extTrigger) {

            tExtDelay = delay/20000;

            if (tExtDelay > 0) {

                uint32_t temp = offsetTimeout + calcTimeOut + tExtDelay;

                bufStartScanExt[1] = (unsigned char)(tExtDelay >> 24);
                bufStartScanExt[2] = (unsigned char)(tExtDelay >> 16);
                bufStartScanExt[3] = (unsigned char)(tExtDelay >> 8);
                bufStartScanExt[4] = (unsigned char)(tExtDelay);
            }

            status = ftdi_write_data(ftdi_dev, bufStartScanExt, 5);
            tc = ftdi_read_data_submit(ftdi_dev, incomingBuf, L << 1);
            ftdi_transfer_data_done(tc);

            if (tc->size > 0) {
                bytesRead = tc->size;
            }
            else {
                std::vector<double> v(3648, 0.0);
                return v;
            }

            ftdi_tcioflush(ftdi_dev);
        }
        else {

            status = ftdi_write_data(ftdi_dev, bufStartScan, 1);
            tc = ftdi_read_data_submit(ftdi_dev, incomingBuf, L << 1);
            ftdi_transfer_data_done(tc);

            if (tc->size > 0) {
                bytesRead = tc->size;
            }
            else {
                std::vector<double> v(3648, 0.0);
                return v;
            }

            ftdi_tcioflush(ftdi_dev);
        }

        uint16_t temp[7388];

        for (int i = 0; i < int(bytesRead >> 1); i++) {

            //Each pixel has 2 bytes of data to form a 16 bit integer
            temp[i] = (uint16_t)(incomingBuf[i << 1] | (incomingBuf[(i << 1) + 1] << 8));

            if (i >= 32 && i <= 3679) {
                yData.push_back(double(temp[i]));
            }
        }

        return yData;

        //return getYDataSequence(extTrigger, 1, 0);
    }

    /** Gets fast series of YData.
      * Returns [nr] vectors where last value is timestamp in ms and the rest are intensity.
      * \param[in] nr number of acquisitions.
      * \param[in] tinterval time interval between subsequent acquisitions in ms.
      * \param[in] extTrigger whether to use external trigger for first acquisition.*/
    std::vector<std::vector<double>> SarspecResDevice::getYDataSequence(bool extTrigger, int nr, int tinterval)
    {

        if (!is_connected) {
            std::vector<double> v(3649, 0.0);
            std::vector<std::vector<double>> V(nr, v);
            return V;
        }

        int status;
        unsigned char bufStartScan[] = { 0x02, 0x00 };
        unsigned char bufStartScanExt[] = { 0x10, 0, 0, 0, 1 };
        unsigned char bufEndScanExt[] = {0x12};
        unsigned char incomingBufs[nr][8192];
        uint32_t bytesRead;
        timespec instants[nr * 2];
        yDataS.clear();

        //First acquisition using external trigger
        if (extTrigger) {

            ftdi_tcioflush(ftdi_dev);

            for(int i = 0; i < nr; i++)
            {
                clock_gettime(CLOCK_MONOTONIC, &instants[i*2]);

                ftdi_write_data(ftdi_dev, bufStartScanExt, 5);

                tc = ftdi_read_data_submit(ftdi_dev, incomingBufs[i], L << 1);
                ftdi_transfer_data_done(tc);

                clock_gettime(CLOCK_MONOTONIC, &instants[i*2+1]);

                ftdi_tcioflush(ftdi_dev);
            }


        }
        //First acquisition using internal trigger
        else {

            for (int i = 0; i < nr; i++) {

                ftdi_tcioflush(ftdi_dev);

                clock_gettime(CLOCK_MONOTONIC, &instants[i * 2]);

                status = ftdi_write_data(ftdi_dev, bufStartScan, 1);

                tc = ftdi_read_data_submit(ftdi_dev, incomingBufs[i], L << 1);
                ftdi_transfer_data_done(tc);

                clock_gettime(CLOCK_MONOTONIC, &instants[i * 2 + 1]);

                clock_gettime(CLOCK_MONOTONIC, &instants[i * 2 + 2]);
                while ((double)(timespecDiff(instants[i * 2], instants[i * 2 + 2]))/1e6 < tinterval) {
                    clock_gettime(CLOCK_MONOTONIC, &instants[i * 2 + 2]);
                }
            }
        }

        if (tc->size > 0) {
                bytesRead = tc->size;
        }
        else {
            std::vector<double> v(3649, 0.0);
            std::vector<std::vector<double>> V(nr, v);
            return V;
        }

        std::vector<double> temp;
        std::vector<uint16_t> temp1;

        for (int j = 0; j < nr; j++) {

            temp1.clear();
            temp.clear();

            for (int i = 0; i < int(bytesRead >> 1); i++) {

                //Each pixel has 2 bytes of data to form a 16 bit integer
                temp1.push_back((uint16_t)(incomingBufs[j][i << 1] | (incomingBufs[j][(i << 1) + 1] << 8)));

                if (i >= 32 && i <= 3679) {
                    temp.push_back(double(temp1[i]));
                }
            }
            temp.push_back((double)(timespecDiff(instants[0], instants[j * 2]))/1e6);
            yDataS.push_back(temp);
        }

        for (int i = 1; i < nr; i++) {
            printf("%u: %g ms, ", i, (double)(timespecDiff(instants[0], instants[i * 2]))/1e6);
        }
        printf("\n");
        for (int i = 1; i < nr; i++) {
            printf("%u - %u: %g ms, ", i - 1, i, (double)(timespecDiff(instants[i * 2 - 2], instants[i * 2]))/1e6);
        }
        printf("\n");

        return yDataS;
    }

    /** Sets device integration time.
      * Returns true on success, false on error.
      * \param[in] intTime new integration time value in ms. Minimum of 4ms.*/
    bool SarspecResDevice::setIntegrationTime(int intTime)
    {

        if (!is_connected)
            return false;

        int status;

        if(intTime < 3 || intTime > 214500){return false;}

        uint32_t delay = (uint32_t)((double)intTime * 20000.0);

        if (delay_old < delay) {
            calcTimeOut = delay / 20000;
        }
        else {
            calcTimeOut = delay_old / 20000;
        }

        if (delay_old == 0) {

            ftdi_dev -> usb_read_timeout = 300000;
            ftdi_dev -> usb_write_timeout = 300000;
        }
        else {

            uint32_t temp = offsetTimeout + calcTimeOut + tExtDelay;
            ftdi_dev -> usb_read_timeout = temp;
            ftdi_dev -> usb_write_timeout = temp;
        }

        unsigned char bufDelay[] = {0x16, 0, 0, 0, 0};

        bufDelay[1] = (unsigned char)(delay >> 24);
        bufDelay[2] = (unsigned char)(delay >> 16);
        bufDelay[3] = (unsigned char)(delay >> 8);
        bufDelay[4] = (unsigned char)(delay);

        status = ftdi_write_data(ftdi_dev, bufDelay, 5);
        delay_old = delay;

        if (status > 0) {return true;}

        return false;
    }

    /** Sets USB port timeout.
      * Returns true on success, false on error.
      * \param[in] timeOut new time out value in ms. Minimum of 100ms*/
    bool SarspecResDevice::setTimeout(int timeOut)
    {

        if (!is_connected)
            return false;

        int status;
        if (timeOut < 100) {return false;}

        offsetTimeout = timeOut;

        uint32_t temp = offsetTimeout + calcTimeOut + tExtDelay;
        ftdi_dev -> usb_read_timeout = temp;
        ftdi_dev -> usb_write_timeout = temp;

        if (status == 0) {return true;}
        return false;
    }

    /** Reads EEPROMPage.
      * Returns read character array.
      * \param[in] page page to be read 0 <= page < 255.*/
    char* SarspecResDevice::readEEPROMPage(int page)
    {
        int status;
        static char ret[64];
        unsigned char bufEEPROM_RCMD[] = { 0x0E, 0x03, 0x00 };
        unsigned char bufEEPROM_R[64];
        unsigned char pageaddr;

        if (page > 255 || !is_connected)
            return ret;

        pageaddr = (unsigned char)page;
        bufEEPROM_RCMD[1] = pageaddr;

        status = ftdi_write_data(ftdi_dev, bufEEPROM_RCMD, 2);
        tc = ftdi_read_data_submit(ftdi_dev, bufEEPROM_R, 64);
        ftdi_transfer_data_done(tc);

        for (int k = 0; k < 64; k++)
        {
            if((char)bufEEPROM_R[k] != (char)0)
            {
                ret[k] = (char)bufEEPROM_R[k];
            }
        }
        ftdi_tcioflush(ftdi_dev);

        return ret;
    }

    /** Sets device dark.
      * Returns dark set.
      * \param[in] dark new dark value.*/
    double SarspecResDevice::setDark(int dark)
    {

        if (!is_connected)
            return 0;

        int status;
        uint16_t DARK = dark;
        unsigned char bufDark[] = {0x06, 0x00, 0x00};

        bufDark[1] = (unsigned char)(DARK >> 8);
        bufDark[2] = (unsigned char)(DARK);

        status = ftdi_write_data(ftdi_dev, bufDark, 3);

        if (status > 0) {
            return double(DARK) / (1000.0 * DA_LEVELS / 4096);
        }

        return 0;
    }

    /** Sets device gain.
      * Returns gain set in V.
      * \param[in] gain new gain value.*/
    double SarspecResDevice::setGain(int gain)
    {

        if (!is_connected)
            return 0;

        int status;
        uint16_t GAIN = gain;

        GAIN = (uint16_t)(GAIN << 2);

        unsigned char bufGain[] = {0x08, 0x00, 0x00};

        bufGain[1] = (unsigned char)(GAIN >> 8);
        bufGain[2] = (unsigned char)(GAIN);

        status = ftdi_write_data(ftdi_dev, bufGain, 3);

        if (status > 0) {
            return 6160.0 / (GAIN / 4.0);
        }

        return 0;
    }

    /** Reads EEPROM dark and gain values.
      * Returns read dark and gain array.*/
    double* SarspecResDevice::getCurrentEEPROMDarkGain()
    {

        if (!is_connected)
            return 0;

        char *tempstr = readEEPROMPage(0);

        //char to string
        std::string initstr(tempstr);

        //split string by #
        std::istringstream ss(initstr);
        std::string token;
        for(int i=0;i<6;i++)
        {
            getline(ss, token, '#');
            if(i==4)
            {
                darkGain[0]= atof(token.c_str());
            }
            if(i==5)
            {
                darkGain[1]= atof(token.c_str());
            }
        }

        return darkGain;
    }

    /** Sets device gain (user).
      * Returns true on success, false on error.
      * \param[in] gain new gain value 0 < gain < 500.*/
    bool SarspecResDevice::setDeviceGain(int gain)
    {

        if (gain < 1 || gain > 500 || !is_connected) {
            return false;
        }

        double gain0inV;

        gain0inV = 6160.0/gain0;
        gain--;

        double temp = std::round(6160.0/(gain + gain0inV));

        int tmp = setGain(int(temp));

        if(tmp==0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    //https://stackoverflow.com/questions/6749621/how-to-create-a-high-resolution-timer-in-linux-to-measure-program-performance
    uint64_t SarspecResDevice::timespecDiff(timespec start, timespec end)
    {
        return ((end.tv_sec * 1000000000UL) + end.tv_nsec) -
            ((start.tv_sec * 1000000000UL) + start.tv_nsec);
    }

} // namespace sarspec_usb
// vim: syntax=cpp ts=4 sw=4 sts=4 sr et
