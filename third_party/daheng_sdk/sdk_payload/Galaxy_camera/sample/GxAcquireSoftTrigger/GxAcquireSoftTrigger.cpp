//-------------------------------------------------------------
/**
\file      GxAcquireSoftTrigger.cpp
\brief     Sample to show how to acquire image by trigger software and save ppm image
\version   1.1.2504.9221
\date      2025.4.22
*/
//-------------------------------------------------------------

#include "../../inc/GxIAPI.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include "../Common/Common.h"

#define ACQ_BUFFER_NUM 5              ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE (64 * 1024) ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64    ///< Qty. of data transfer block
#define FILE_NAME_LEN 50              ///< Save image file name length

#define PIXFMT_CVT_FAIL -1   ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS 0 ///< PixelFormatConvert success

// Show error message
#define GX_VERIFY(emStatus)            \
    if (emStatus != GX_STATUS_SUCCESS) \
    {                                  \
        GetErrorString(emStatus);      \
        return emStatus;               \
    }

// Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus)       \
    if (emStatus != GX_STATUS_SUCCESS) \
    {                                  \
        GetErrorString(emStatus);      \
        GXCloseDevice(g_hDevice);      \
        g_hDevice = NULL;              \
        GXCloseLib();                  \
        printf("<App Exit!>\n");       \
        return emStatus;               \
    }

GX_DEV_HANDLE g_hDevice = NULL;                  ///< Device handle
bool g_bColorPixelFormat = false;                ///< Color pixel format
bool g_bAcquisitionFlag = false;                 ///< Thread running flag
pthread_t g_nAcquisitonThreadID = 0;             ///< Thread ID of Acquisition thread

unsigned char *g_pDesImageBuf = NULL;  ///< Memory for desimage(Mono image / RGB24)
uint32_t g_nDesImageLen = 0;             ///< Memory length for desimage

uint32_t g_nPayloadSize = 0; ///< Payload size

// Allocate the memory for pixel format transform
void PreForAcquisition();

// Release the memory allocated
void UnPreForAcquisition();

// Save one frame to PPM image file
void SavePPMFile(uint32_t, uint32_t);

// Acquisition thread function
void *ProcGetImage(void *);

// Get description of error
void GetErrorString(GX_STATUS);

int main()
{
    printf("\n");
    printf("-------------------------------------------------------------\n");
    printf("Sample to show how to acquire image by trigger software and save ppm image.\n");
    printf("version: 1.1.2604.9011\n");
    printf("-------------------------------------------------------------\n");
    printf("\n");
    printf("<Initializing......>");
    printf("\n\n");

    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    uint32_t ui32DeviceNum = 0;

    // Initialize libary
    emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    // Get device enumerated number
    emStatus = GXUpdateAllDeviceList(&ui32DeviceNum, 1000);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }

    // If no device found, app exit
    if (ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return emStatus;
    }

    // Open first device enumerated
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }

    // Get Device Info
    printf("***********************************************\n");
    // Get libary version
    printf("<Libary Version : %s>\n", GXGetLibVersion());

    // Get string length of Vendor name
    GX_STRING_VALUE stStrVendorName;
    emStatus = GXGetStringValue(g_hDevice, "DeviceVendorName", &stStrVendorName);
    GX_VERIFY_EXIT(emStatus);
    printf("<Vendor Name : %s>\n", stStrVendorName.strCurValue);

    // Get string length of Model name
    GX_STRING_VALUE stStrModelName;
    emStatus = GXGetStringValue(g_hDevice, "DeviceModelName", &stStrModelName);
    GX_VERIFY_EXIT(emStatus);
    printf("<Model Name : %s>\n", stStrModelName.strCurValue);

    // Get string length of Serial number
    GX_STRING_VALUE stStrSerialNumber;
    emStatus = GXGetStringValue(g_hDevice, "DeviceSerialNumber", &stStrSerialNumber);
    GX_VERIFY_EXIT(emStatus);
    printf("<Serial Number : %s>\n", stStrSerialNumber.strCurValue);

    // Get string length of Device version
    GX_STRING_VALUE stStrDeviceVersion;
    emStatus = GXGetStringValue(g_hDevice, "DeviceVersion", &stStrDeviceVersion);
    GX_VERIFY_EXIT(emStatus);
    printf("<Device Version : %s>\n", stStrDeviceVersion.strCurValue);

    printf("***********************************************\n");

    uint32_t nDSNum = 0;
    emStatus = GXGetDataStreamNumFromDev(g_hDevice, &nDSNum);
    GX_VERIFY_EXIT(emStatus);

    if (nDSNum < 1)
    {
        printf("Failed to obtain the number of streams\n");
        return GX_STATUS_ERROR;
    }

    GX_DS_HANDLE phDS = NULL;
    emStatus = GXGetDataStreamHandleFromDev(g_hDevice, 1, &phDS);
    GX_VERIFY_EXIT(emStatus);
    if (NULL == phDS)
    {
        printf("Failed to get data stream handle\n");
        return GX_STATUS_INVALID_HANDLE;
    }

    emStatus = GXGetPayLoadSize(phDS, &g_nPayloadSize);
    GX_VERIFY_EXIT(emStatus);

    printf("\n");
    printf("Press [a] or [A] and then press [Enter] to start acquisition\n");
    printf("Press [s] or [S] and then press [Enter] to send a soft trigger and save one ppm image\n");
    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("\n");

    int nStartKey = 0;
    bool bWaitStart = true;
    while (bWaitStart)
    {
        nStartKey = getchar();
        switch (nStartKey)
        {
        // press 'a' and [Enter] to start acquisition
        // press 'x' and [Enter] to exit
        case 'a':
        case 'A':
            // Start to acquisition
            bWaitStart = false;
            break;
        case 'S':
        case 's':
            printf("<Please start acquisiton before sending soft trigger!>\n");
            break;
        case 'x':
        case 'X':
            // App exit
            GXCloseDevice(g_hDevice);
            g_hDevice = NULL;
            GXCloseLib();
            printf("<App exit!>\n");
            return 0;
        default:
            break;
        }
    }

    // Set PacketSize
    GX_NODE_ACCESS_MODE emAccessMode;
    emStatus = GXGetNodeAccessMode(g_hDevice, "GevSCPSPacketSize", &emAccessMode);
    bool bImplementPacketSize = (emAccessMode == GX_NODE_ACCESS_MODE_RW) ? true : false;
    if (bImplementPacketSize)
    {
        uint32_t ui32PacketSize = 0;
        emStatus = GXGetOptimalPacketSize(g_hDevice, &ui32PacketSize);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            printf("<Failed to get OptimalPacketSize!>");
        }
        emStatus = GXSetIntValue(g_hDevice, "GevSCPSPacketSize", ui32PacketSize);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            printf("<Failed to set GevSCPSPacketSize!>");
        }
    }

    // Set PacketDelay
    emStatus = GXGetNodeAccessMode(g_hDevice, "GevSCPD", &emAccessMode);
    bool bImplementPacketDelay = (emAccessMode == GX_NODE_ACCESS_MODE_RW) ? true : false;
    if (bImplementPacketDelay)
    {
        GX_INT_VALUE stValue;
        emStatus = GXGetIntValue(g_hDevice, "GevSCPD", &stValue);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            printf("<Failed to get GevSCPD!>");
        }
        emStatus = GXSetIntValue(g_hDevice, "GevSCPD", stValue.nMin);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            printf("<Failed to set GevSCPD!>");
        }
    }

    // Set acquisition mode
    emStatus = GXSetEnumValueByString(g_hDevice, "AcquisitionMode", "Continuous");
    GX_VERIFY_EXIT(emStatus);

    emStatus = GXGetNodeAccessMode(g_hDevice, "TriggerSelector", &emAccessMode);
    GX_VERIFY_EXIT(emStatus);

    if( GX_NODE_ACCESS_MODE_RW == emAccessMode)
    {
        // Set trigger selector
        emStatus = GXSetEnumValueByString(g_hDevice, "TriggerSelector", "FrameStart");
        GX_VERIFY_EXIT(emStatus);
    }

    // Set trigger mode
    emStatus = GXSetEnumValueByString(g_hDevice, "TriggerMode", "On");
    GX_VERIFY_EXIT(emStatus);

    // Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
    GX_VERIFY_EXIT(emStatus);

    // Get the type of pixel format. whether is a color pixel format.
    GX_ENUM_VALUE emValue;
    emStatus = GXGetEnumValue(g_hDevice, "PixelFormat", &emValue);
    GX_VERIFY_EXIT(emStatus);

    g_bColorPixelFormat = !IsGrayPixelFormat(( GX_PIXEL_FORMAT_ENTRY)emValue.stCurValue.nCurValue);

    GX_NODE_ACCESS_MODE emStreamTransferSizeIm;
    emStatus = GXGetNodeAccessMode(phDS, "StreamTransferSize", &emStreamTransferSizeIm);
    GX_VERIFY_EXIT(emStatus);
    bool bStreamTransferSize = ((emStreamTransferSizeIm == GX_NODE_ACCESS_MODE_RO) || (emStreamTransferSizeIm == GX_NODE_ACCESS_MODE_WO) || (emStreamTransferSizeIm == GX_NODE_ACCESS_MODE_RW)) ? true : false;
    if (bStreamTransferSize)
    {
        // Set size of data transfer block
        emStatus = GXSetIntValue(phDS, "StreamTransferSize", ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(emStatus);
    }

    GX_NODE_ACCESS_MODE emStreamTransferUrbIm;
    emStatus = GXGetNodeAccessMode(phDS, "StreamTransferNumberUrb", &emStreamTransferUrbIm);
    GX_VERIFY_EXIT(emStatus);
    bool bStreamTransferNumberUrb = ((emStreamTransferUrbIm == GX_NODE_ACCESS_MODE_RO) || (emStreamTransferUrbIm == GX_NODE_ACCESS_MODE_WO) || (emStreamTransferUrbIm == GX_NODE_ACCESS_MODE_RW)) ? true : false;
    if (bStreamTransferNumberUrb)
    {
        // Set qty. of data transfer block
        emStatus = GXSetIntValue(phDS, "StreamTransferNumberUrb", ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(emStatus);
    }

    // Prepare for Acquisition, alloc memory for image pixel format tansform
    PreForAcquisition();

    // Device start acquisition
    emStatus = GXStreamOn(g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        // Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    // Start acquisition thread, if thread create failed, exit this app
    int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, NULL);
    if (nRet != 0)
    {
        // Release the memory allocated
        UnPreForAcquisition();

        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();

        printf("<Failed to create the acquisition thread, App Exit!>\n");
        exit(nRet);
    }

    // Main loop
    bool bRun = true;
    while (bRun == true)
    {
        int nKey = getchar();
        // press 's' and [Enter] for save image
        // press 'x' and [Enter] for exit
        switch (nKey)
        {
        // Send a software trigger and save PPM Image
        case 'S':
        case 's':
            emStatus = GXSetCommandValue(g_hDevice, "TriggerSoftware");
            if (emStatus != GX_STATUS_SUCCESS)
            {
                GetErrorString(emStatus);
            }
            break;
        // Exit app
        case 'X':
        case 'x':
            bRun = false;
            break;
        default:
            break;
        }
    }

    // Stop Acquisition thread
    g_bAcquisitionFlag = false;
    pthread_join(g_nAcquisitonThreadID, NULL);

    // Device stop acquisition
    emStatus = GXStreamOff(g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        // Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    // Release the memory allocated
    UnPreForAcquisition();

    // Close device
    emStatus = GXCloseDevice(g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();
        return emStatus;
    }

    // Release libary
    emStatus = GXCloseLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    printf("<App exit!>\n");
    return 0;
}

//-------------------------------------------------
/**
\brief Save PPM image
\param ui32Width[in]       image width
\param ui32Height[in]      image height
\return void
*/
//-------------------------------------------------
void SavePPMFile(uint32_t ui32Width, uint32_t ui32Height)
{
    char szName[FILE_NAME_LEN] = {0};

    static int nRawFileIndex = 0;
    FILE *phImageFile = NULL;

    if (g_pDesImageBuf != NULL)
    {
        snprintf(szName, FILE_NAME_LEN, "Frame_%d.ppm", nRawFileIndex++);
        phImageFile = fopen(szName, "wb");
        if (phImageFile == NULL)
        {
            printf("Create or Open %s failed!\n", szName);
            return;
        }

        if (g_bColorPixelFormat)
        {
            fprintf(phImageFile, "P6\n%u %u 255\n", ui32Width, ui32Height);
        }
        else
        {
            fprintf(phImageFile, "P5\n%u %u 255\n", ui32Width, ui32Height);
        }
        
        fwrite(g_pDesImageBuf, 1, g_nDesImageLen, phImageFile);
        fclose(phImageFile);
        phImageFile = NULL;
        printf("Save %s succeed\n", szName);
    }
    else
    {
        printf("Save %s failed!\n", szName);
    }
    return;
}

//-------------------------------------------------
/**
\brief Allocate the memory for pixel format transform
\return void
*/
//-------------------------------------------------
void PreForAcquisition()
{
    // Calculate the required buffer length for image conversion.
    if (g_bColorPixelFormat)
    {
        g_nDesImageLen = g_nPayloadSize * 3;
    }
    else
    {
        g_nDesImageLen = g_nPayloadSize;
    }

    // Alloc memory for des image pixel format transform
    g_pDesImageBuf = new unsigned char[g_nDesImageLen];

    return;
}

//-------------------------------------------------
/**
\brief Release the memory allocated
\return void
*/
//-------------------------------------------------
void UnPreForAcquisition()
{
    // Release resources
    if (g_pDesImageBuf != NULL)
    {
        delete[] g_pDesImageBuf;
        g_pDesImageBuf = NULL;
    }

    return;
}

//-------------------------------------------------
/**
\brief Acquisition thread function
\param pParam[in]       thread param, not used in this app
\return void*
*/
//-------------------------------------------------
void *ProcGetImage(void *pParam)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Thread running flag setup
    g_bAcquisitionFlag = true;
    PGX_FRAME_DATA_EX pFrameBuffer = NULL;

    while (g_bAcquisitionFlag)
    {
        // Get a frame from Queue
        emStatus = GXDQBufEx(g_hDevice, &pFrameBuffer, 1000);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                GetErrorString(emStatus);
                break;
            }
        }
        if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }
        else
        {
            printf("<Successful acquisition: Width: %d Height: %d FrameID: %lu>\n",
                   pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
            
            GX_PIXEL_FORMAT_ENTRY emDesFormat = (g_bColorPixelFormat)? GX_PIXEL_FORMAT_RGB8:GX_PIXEL_FORMAT_MONO8;
            g_nDesImageLen = ( g_bColorPixelFormat)? g_nPayloadSize * 3: g_nPayloadSize;
            int nRet = PixelFormatConvert(pFrameBuffer, g_pDesImageBuf, &g_nDesImageLen, emDesFormat);
            if (nRet == PIXFMT_CVT_SUCCESS)
            {
                SavePPMFile(pFrameBuffer->nWidth, pFrameBuffer->nHeight);
            }
            else
            {
                printf("PixelFormat Convert failed!\n");
            }
        }

        emStatus = GXQBufEx(g_hDevice, pFrameBuffer);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            break;
        }
    }
    printf("<Acquisition thread Exit!>\n");

    return 0;
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];

    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>>\n");
    }
    else
    {
        printf("%s\n", error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete[] error_info;
        error_info = NULL;
    }
}
