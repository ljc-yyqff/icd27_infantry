//-------------------------------------------------------------
/**
\file      GxCXPCardSample.cpp
\brief     Sample to show how to use cxp card.
\version   1.1.2507.9141
\date      2025.07.14
*/
//-------------------------------------------------------------
#include "../../../inc/GxIAPI.h"
#include "../../Common/Common.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include <string.h>

using namespace std;
void GetErrorString(GX_STATUS emErrorStatus);

#define FILE_NAME_LEN 50     ///< Save image file name length
#define PIXFMT_CVT_FAIL -1   ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS 0 ///< PixelFormatConvert success

//Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
        return emStatus;                   \
    }

GX_DEV_HANDLE g_hDevice = NULL;           	///< Device handle
uint32_t g_nDesImageLen = 0;             	///< Memory length for desimage
bool g_bColorPixelFormat = false;           ///< Color pixel format
uint32_t g_nPayloadSize = 0; 				///< Payload size

//-------------------------------------------------
/**
\brief Save PPM image
\param pDesImageBuf        Memory for desimage(Mono image / RGB24)
\param ui32Width[in]       image width
\param ui32Height[in]      image height
\return void
*/
//-------------------------------------------------
void SavePPMFile(unsigned char *pDesImageBuf, uint32_t ui32Width, uint32_t ui32Height)
{
    char szName[FILE_NAME_LEN] = {0};

    static int nRawFileIndex = 0;
    FILE *phImageFile = NULL;

    if (pDesImageBuf != NULL)
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
        
        fwrite(pDesImageBuf, 1, g_nDesImageLen, phImageFile);
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

int main(int argc, char* argv[])
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Init library
    emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return 0;
    }

    //Get device enumerated number
    uint32_t ui32DeviceNum = 0;
    emStatus = GXUpdateAllDeviceList(&ui32DeviceNum, 1000);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return 0;
    }

    //If no device found, app exit
    if (ui32DeviceNum <= 0)
    {
        cout << "No device!" << endl;
        GXCloseLib();
        return 0;
    }

    //Open first device enumerated
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return 0;
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
    
    // CXP card interface function demonstration
    GX_IF_HANDLE hIF = NULL;
    emStatus = GXGetParentInterfaceFromDev(g_hDevice, &hIF);
    GX_VERIFY_EXIT(emStatus);
    
    // Retrieve and print the card display name and type
    GX_STRING_VALUE strVal;
    memset(&strVal, 0, sizeof(GX_STRING_VALUE));
    emStatus = GXGetStringValue(hIF, "InterfaceDisplayName", &strVal);
    GX_VERIFY_EXIT(emStatus);
    printf("InterfaceDisplayName: %s\n", strVal.strCurValue);

    GX_ENUM_VALUE stEnumValue;
    memset(&stEnumValue, 0, sizeof(GX_ENUM_VALUE));
    emStatus = GXGetEnumValue(hIF, "InterfaceType", &stEnumValue);
    GX_VERIFY_EXIT(emStatus);
    printf("InterfaceType: %s\n", stEnumValue.stCurValue.strCurSymbolic);

    // Set line5 of the card to output a high level
    emStatus = GXSetEnumValueByString(hIF, "UserOutputSelector", "UserOutput5");
    GX_VERIFY_EXIT(emStatus);
    emStatus |= GXSetBoolValue(hIF, "UserOutputValue", true);
    GX_VERIFY_EXIT(emStatus);
    emStatus = GXSetEnumValueByString(hIF, "LineSelector", "Line5");
    GX_VERIFY_EXIT(emStatus);
    emStatus = GXSetEnumValueByString(hIF, "LineMode", "Output");
    GX_VERIFY_EXIT(emStatus);
    emStatus = GXSetEnumValueByString(hIF, "LineSource", "UserOutput5");
    GX_VERIFY_EXIT(emStatus);

    // Get the type of pixel format. whether is a color pixel format.
    GX_ENUM_VALUE emValue;
    emStatus = GXGetEnumValue(g_hDevice, "PixelFormat", &emValue);
    GX_VERIFY_EXIT(emStatus);

    g_bColorPixelFormat = !IsGrayPixelFormat(( GX_PIXEL_FORMAT_ENTRY)emValue.stCurValue.nCurValue);

    GX_LOCAL_DEV_HANDLE hLocalDev = NULL;
    // If it is a color camera, image interpolation and red blue swap can be achieved through the card
    if (g_bColorPixelFormat)
    {
        // Local interface function demonstration
        emStatus = GXGetLocalDeviceHandleFromDev(g_hDevice, &hLocalDev);
        GX_VERIFY_EXIT(emStatus);

        emStatus = GXSetEnumValueByString(hLocalDev, "BayerConversion", "Enable");
        GX_VERIFY_EXIT(emStatus);

        GX_ENUM_VALUE stEnumValue;
        emStatus = GXGetEnumValue(hLocalDev, "OutPixelFormat", &stEnumValue);
        GX_VERIFY_EXIT(emStatus);
        printf("OutPixelFormat: %s\n", stEnumValue.stCurValue.strCurSymbolic);

        emStatus = GXSetEnumValueByString(hLocalDev, "RedBlueSwap", "Disable");
        GX_VERIFY_EXIT(emStatus);
    }
    else
    {
        emStatus = GXSetEnumValueByString(g_hDevice, "PixelFormat", "Mono8");
        GX_VERIFY_EXIT(emStatus);
    }
    
    GX_FLOAT_VALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(stFloatValue));
    emStatus = GXGetFloatValue(g_hDevice, "ExposureTime", &stFloatValue);
    GX_VERIFY_EXIT(emStatus);
    printf("ExposureTime: %.2f\n", stFloatValue.dCurValue);

    //Device start acquisition
    emStatus = GXSetCommandValue(g_hDevice, "AcquisitionStart");
    GX_VERIFY_EXIT(emStatus);

    PGX_FRAME_DATA_EX pFrameBuffer = NULL;
    emStatus = GXDQBufEx(g_hDevice, &pFrameBuffer, 1000);
    if (emStatus == GX_STATUS_SUCCESS)
    {
        if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }
        else
        {
            GX_PIXEL_FORMAT_ENTRY emDesFormat = (g_bColorPixelFormat)? GX_PIXEL_FORMAT_RGB8:GX_PIXEL_FORMAT_MONO8;
            g_nDesImageLen = ( g_bColorPixelFormat)? g_nPayloadSize * 3: g_nPayloadSize;
            unsigned char *pDesImageBuf = new unsigned char[g_nDesImageLen];
            int nRet = PixelFormatConvert(pFrameBuffer, pDesImageBuf, &g_nDesImageLen, emDesFormat);
            if (nRet == PIXFMT_CVT_SUCCESS)
            {
                SavePPMFile(pDesImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight);
            }
            else
            {
                printf("PixelFormat Convert failed!\n");
            }
            delete[] pDesImageBuf;
            pDesImageBuf = NULL;
        }

        GXQBufEx(g_hDevice, pFrameBuffer);
    }

    //Device stop acquisition
    GXSetCommandValue(g_hDevice, "AcquisitionStop");

    //Disable BayerConversion
    if (g_bColorPixelFormat)
    {
        GXSetEnumValueByString(hLocalDev, "BayerConversion", "Disable");
    }

    //Close device
    GXCloseDevice(g_hDevice);
    //Release libary
    GXCloseLib();

    cout << "<App exit!>" << endl;

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
    if(emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }
    
    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
    
    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}
