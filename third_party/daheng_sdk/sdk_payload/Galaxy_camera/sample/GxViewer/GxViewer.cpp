//--------------------------------------------------------------------------------
/**
\file     GxViewer.cpp
\brief    CGxViewer Class implementation file

\version  v1.0.2505.9191
\date     2025-05-19

<p>Copyright (c) 2017-2025</p>
*/
//----------------------------------------------------------------------------------
#include "GxViewer.h"
#include "ui_GxViewer.h"

#define DEFAULT_PROP_WIDTH  450
#define MIN_PROP_HEIGHT     600

//----------------------------------------------------------------------------------
/**
\Constructor of CGxViewer
*/
//----------------------------------------------------------------------------------
CGxViewer::CGxViewer(QWidget *parent) : QMainWindow(parent),
                                        ui(new Ui::CGxViewer),
                                        m_pobjAcqThread(NULL),
                                        m_pobjImgProc(NULL),
                                        m_hDevice(NULL),
                                        m_pstBaseInfo(NULL),
                                        m_ui32DeviceNum(0),
                                        m_bOpen(false),
                                        m_bAcquisitionStart(false),
                                        m_bColorFilter(false),
                                        m_bSaveImage(false),
                                        m_ui32ShowCount(0),
                                        m_dImgShowFrameRate(0.0),
                                        m_pobjShowImgTimer(NULL),
                                        m_pobjShowFrameRateTimer(NULL),
                                        m_hProperty(NULL)
{
    ui->setupUi(this);

    this->__SetSystemIcon();

    __SetKeyboardShortCut();

    // Customize type using signal-slot must registed
    qRegisterMetaType<VxInt32>("VxInt32");

    // Setup Image show timer and Frame rate show timer
    m_pobjShowImgTimer = new QTimer(this);
    m_pobjShowFrameRateTimer = new QTimer(this);
    connect(m_pobjShowImgTimer, SIGNAL(timeout()), this, SLOT(slotShowImage()));
    connect(m_pobjShowFrameRateTimer, SIGNAL(timeout()), this, SLOT(slotShowFrameRate()));

    // Connect image save signal and slot non-blocking
    connect(this, SIGNAL(SigSaveImage()), this, SLOT(slotSaveImageFile()), Qt::QueuedConnection);

    // Init GxiApi libary
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
    }

    UpdateUI();
}

//----------------------------------------------------------------------------------
/**
\Destructor of CGxViewer
*/
//----------------------------------------------------------------------------------
CGxViewer::~CGxViewer()
{
    // Stop acquisition thread if acquisition still running
    if (m_bAcquisitionStart)
    {
        m_pobjAcqThread->m_bAcquisitionThreadFlag = false;
        m_pobjAcqThread->quit();
        m_pobjAcqThread->wait();
    }

    // Release acquisition thread object
    RELEASE_ALLOC_MEM(m_pobjAcqThread);

    RELEASE_ALLOC_MEM(m_pobjShowImgTimer);
    RELEASE_ALLOC_MEM(m_pobjShowFrameRateTimer);

    // Release baseinfo
    RELEASE_ALLOC_ARR(m_pstBaseInfo);

    // Release Dialogs
    DestroyDialogs();

    // Release GxiApi libary
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXCloseLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
    }

    delete ui;
}

//-------------------------------------------------------------------------
/**
\Set the system icon
\return void
*/
//-------------------------------------------------------------------------
void CGxViewer::__SetSystemIcon()
{
    // Get the exe file path
    QString string_path = QCoreApplication::applicationDirPath();

    // Get the icon path
    QString string_con_path = string_path + QString(":/resources/logo.png");

    // Read the icon
    QPixmap objIconImg(string_con_path);

    // If the icon load fails, the default icon is used.
    if (objIconImg.isNull())
    {
        this->setWindowIcon(QIcon(":/resources/logo.png"));
    }
    else // Load the other icon
    {
        this->setWindowIcon(QIcon(objIconImg));
    }

    return;
}

//-------------------------------------------------------------------------
/**
\Set Keyboard ShortCut
\return void
*/
//-------------------------------------------------------------------------
void CGxViewer::__SetKeyboardShortCut()
{
    // Set shortcut of some pushbutton
    ui->UpdateDeviceList->setShortcut(QKeySequence(QLatin1String("Ctrl+U")));
    ui->OpenDevice->setShortcut(QKeySequence(QLatin1String("Ctrl+O")));
    ui->CloseDevice->setShortcut(QKeySequence(QLatin1String("Ctrl+C")));
    ui->StartAcquisition->setShortcut(QKeySequence(QLatin1String("Ctrl+A")));
    ui->StopAcquisition->setShortcut(QKeySequence(QLatin1String("Ctrl+P")));
    ui->ImageImprovement->setShortcut(QKeySequence(QLatin1String("Ctrl+I")));
    ui->actionSaveImage->setShortcut(QKeySequence(QLatin1String("Ctrl+S")));

    return;
}
//----------------------------------------------------------------------------------
/**
\Clear ComboBox Items, close child dialogs
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::ClearUI()
{
    if (m_pobjImgProc->isVisible())
    {
        m_pobjImgProc->close();
    }

    // Show device information on Info label
    ui->VendorName->setText(QString("<No Device Opened>"));
    ui->ModelName->clear();
    ui->SerialNumber->clear();
    ui->DeviceVersion->clear();

    // Reset acquisition frame rate to 0
    ui->AcqFrameRateLabel->setText(QString("Frame NUM: %1   Acq. FPS: %2")
                                       .arg(0)
                                       .arg(0.0, 0, 'f', 1));
    // Reset display frame rate
    ui->ShowFrameRateLabel->setText(QString("Disp.FPS: %1").arg(0.0, 0, 'f', 1));

    // Clear show image label
    ui->ImageLabel->clear();

    return;
}

//----------------------------------------------------------------------------------
/**
\ Enable all UI Groups except Camera select group
\param[in]
\param[out]
\return  void
*/
//----------------------------------------------------------------------------------
void CGxViewer::EnableUI()
{
    ui->Capture_Control->setEnabled(true);
}

//----------------------------------------------------------------------------------
/**
\ Disable all UI Groups except Camera select group
\param[in]
\param[out]
\return  void
*/
//----------------------------------------------------------------------------------
void CGxViewer::DisableUI()
{
    ui->Capture_Control->setEnabled(false);
}

//----------------------------------------------------------------------------------
/**
\Update UI items
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::UpdateUI()
{
    ui->UpdateDeviceList->setEnabled(!m_bOpen);
    ui->DeviceList->setEnabled(!m_bOpen);
    ui->OpenDevice->setEnabled(ui->DeviceList->count() > 0 && !m_bOpen);
    ui->CloseDevice->setEnabled(m_bOpen);
    ui->Capture_Control->setEnabled(m_bOpen);
    ui->actionSaveImage->setEnabled(m_bOpen);
    ui->menuSaveImage->setEnabled((m_bAcquisitionStart));
    ui->StartAcquisition->setEnabled(!m_bAcquisitionStart);
    ui->StopAcquisition->setEnabled(m_bAcquisitionStart);
}

//----------------------------------------------------------------------------------
/**
\Setup acquisition thread
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
int CGxViewer::SetUpAcquisitionThread()
{
    // if Acquisition thread is on Stop acquisition thread
    if (m_pobjAcqThread != NULL)
    {
        m_pobjAcqThread->m_bAcquisitionThreadFlag = false;
        m_pobjAcqThread->quit();
        m_pobjAcqThread->wait();

        // Release acquisition thread object
        RELEASE_ALLOC_MEM(m_pobjAcqThread);
    }

    // Instantiation acquisition thread
    try
    {
        m_pobjAcqThread = new CAcquisitionThread;
    }
    catch (std::bad_alloc &e)
    {
        QMessageBox::about(NULL, "Allocate memory error", "Cannot allocate memory, please exit this app!");
        RELEASE_ALLOC_MEM(m_pobjAcqThread);
        return -1;
    }
    catch (std::exception &e)
    {
        QMessageBox::about(NULL, "Error", e.what());
        RELEASE_ALLOC_MEM(m_pobjAcqThread);
        return -1;
    }

    // Connect error signal and error handler
    connect(m_pobjAcqThread, SIGNAL(SigAcquisitionError(QString)), this,
            SLOT(slotAcquisitionErrorHandler(QString)), Qt::QueuedConnection);
    connect(m_pobjAcqThread, SIGNAL(SigImageProcError(VxInt32)), this,
            SLOT(slotImageProcErrorHandler(VxInt32)), Qt::QueuedConnection);

    return 0;
}
//----------------------------------------------------------------------------------
/**
\Setup all dialogs and connet get param
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::SetUpDialogs()
{
    // Release dialogs
    DestroyDialogs();

    try
    {
        // Instantiation dialogs
        m_pobjImgProc = new CImageImprovement;
    }
    catch (std::bad_alloc &e)
    {
        QMessageBox::about(NULL, "Allocate memory error", "Cannot allocate memory, please exit this app!");
        DestroyDialogs();
        return;
    }

    // Connect image processing parameters to acquisition-thread class
    connect(m_pobjImgProc, SIGNAL(SigSendColorCorrectionParam(int64_t)),
            m_pobjAcqThread, SLOT(SlotGetColorCorrectionParam(int64_t)));
    connect(m_pobjImgProc, SIGNAL(SigSendGammaLUT(unsigned char *)),
            m_pobjAcqThread, SLOT(SlotGetGammaLUT(unsigned char *)));
    connect(m_pobjImgProc, SIGNAL(SigSendContrastLUT(unsigned char *)),
            m_pobjAcqThread, SLOT(SlotGetContrastLUT(unsigned char *)));

    return;
}

//----------------------------------------------------------------------------------
/**
\Close all dialogs
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::CloseDialogs()
{
    m_pobjImgProc->close();

    return;
}

//----------------------------------------------------------------------------------
/**
\Destroy all Dialogs
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::DestroyDialogs()
{
    RELEASE_ALLOC_MEM(m_pobjImgProc);

    return;
}

//----------------------------------------------------------------------------------
/**
\Open ImageImprovement dialog(slot)
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_ImageImprovement_clicked()
{
    // Close all dialogs
    CloseDialogs();

    // Get dialog initial param
    m_pobjImgProc->GetDialogInitParam(m_hDevice);

    // Make dialog size unchangable
    m_pobjImgProc->setFixedSize(m_pobjImgProc->width(), m_pobjImgProc->height());

    // Make active window can be modify by click
    m_pobjImgProc->setWindowModality(Qt::WindowModal);

    // Display dialog and raise it to the front
    m_pobjImgProc->show();
    m_pobjImgProc->raise();
    m_pobjImgProc->activateWindow();

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\brief  Save Image menu clicked slot
\param[in]
\param[out]
\return
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_actionSaveImage_triggered()
{
    // If acquisition not started, do nothing
    if (!m_bAcquisitionStart)
    {
        return;
    }

    // Set Save image flag, waiting next showtimer timeout to save image
    m_bSaveImage = true;

    return;
}

//----------------------------------------------------------------------------------
/**
\brief  Save image to customize dir(slot)
\param[in]
\param[out]
\return
*/
//----------------------------------------------------------------------------------
void CGxViewer::slotSaveImageFile()
{
    // If acquisition not started, do nothing
    if (!m_bAcquisitionStart)
    {
        return;
    }

    // Save image to selected or input path
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Save Image"));
    fileDialog->setDirectory(tr("."));
    fileDialog->setDefaultSuffix(tr("bmp"));
    fileDialog->setNameFilter(tr("Image Files(*.bmp *.png)"));
    fileDialog->setAcceptMode(QFileDialog::AcceptSave);

    if (fileDialog->exec() == QDialog::Accepted)
    {
        QString path = fileDialog->selectedFiles()[0];

        if (path.endsWith(".bmp") || path.endsWith(".png"))
        {
            if (!(m_objImageForSave.save(path)))
            {
                QMessageBox::about(this, tr("Save image"), tr("Save image faild!"));
            }
        }
        else
        {
            QMessageBox::about(this, tr("Save image"), tr("Invalid suffix!"));
        }
    }
    fileDialog->close();

    // Release fileDialog
    RELEASE_ALLOC_MEM(fileDialog);

    return;
}

//----------------------------------------------------------------------------------
/**
\brief  Show library version and demo version
\param[in]
\param[out]
\return
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_actionAbout_triggered()
{
    QString szLibVersion = GXGetLibVersion();
    QMessageBox::about(this, tr("About"), tr("Copyright (C)  2015-2025\n\nLibVer. : %1\nDemoVer. : %2").arg(szLibVersion).arg(tr("v1.1.2506.8301")));

    return;
}

//----------------------------------------------------------------------------------
/**
\brief  Enumerate Devcie List, Get baseinfo of these devices
\param[in]
\param[out]
\return
*/
//----------------------------------------------------------------------------------
void CGxViewer::UpdateDeviceList()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // If base info exist, delete it firstly
    RELEASE_ALLOC_ARR(m_pstBaseInfo);

    // Enumerate Devcie List
    emStatus = GXUpdateDeviceList(&m_ui32DeviceNum, ENUMRATE_TIME_OUT);
    GX_VERIFY(emStatus);

    // If avalible devices enumerated, get base info of enumerate devices
    if (m_ui32DeviceNum > 0)
    {
        // Alloc resourses for device baseinfo
        try
        {
            m_pstBaseInfo = new GX_DEVICE_BASE_INFO[m_ui32DeviceNum];
        }
        catch (std::bad_alloc &e)
        {
            QMessageBox::about(NULL, "Allocate memory error", "Cannot allocate memory, please exit this app!");
            RELEASE_ALLOC_MEM(m_pstBaseInfo);
            return;
        }
        // Set size of function "GXGetAllDeviceBaseInfo"
        size_t nSize = m_ui32DeviceNum * sizeof(GX_DEVICE_BASE_INFO);

        // Get all device baseinfo
        emStatus = GXGetAllDeviceBaseInfo(m_pstBaseInfo, &nSize);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            RELEASE_ALLOC_ARR(m_pstBaseInfo);
            ShowErrorString(emStatus);

            // Reset device number
            m_ui32DeviceNum = 0;

            return;
        }
    }

    return;
}

//----------------------------------------------------------------------------------
/**
\brief  Enumerate Devices(slot)
\param[in]
\param[out]
\return
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_UpdateDeviceList_clicked()
{
    ui->DeviceList->clear();

    // Enumerate Devices
    UpdateDeviceList();

    // If enumerate no device, return
    if (m_ui32DeviceNum == 0)
    {
        // Update Mainwindow
        UpdateUI();
        return;
    }

    // Add items and display items on ComboBox
    for (uint32_t i = 0; i < m_ui32DeviceNum; i++)
    {
        QString szDeviceDisplayName = QString::fromUtf8(m_pstBaseInfo[i].szDisplayName);
        ui->DeviceList->addItem(QString(szDeviceDisplayName));
    }

    // Focus on the first device
    ui->DeviceList->setCurrentIndex(0);

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Open Device
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::OpenDevice()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXOpenDeviceByIndex(ui->DeviceList->currentIndex() + 1, &m_hDevice);
    GX_VERIFY(emStatus);

    emStatus = GXCreateWnd(m_hDevice, GX_WND_PROP, NULL, &m_hProperty);
    if (GX_STATUS_SUCCESS != emStatus)
    {
        ShowErrorString(emStatus);
        GXCloseDevice(m_hDevice);
        return;
    }

    // isOpen flag set true
    m_bOpen = true;

    return;
}

//----------------------------------------------------------------------------------
/**
\Open Device(slot)
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_OpenDevice_clicked()
{
    // Open Device
    OpenDevice();

    // Do not init device or get init params when open failed
    if (!m_bOpen)
    {
        return;
    }

    // Setup acquisition thread
    if (0 != SetUpAcquisitionThread())
    {
        CloseDevice();
        return;
    }

    // Setup all Dialogs
    SetUpDialogs();

    // Transfer Device handle to acquisition thread class
    m_pobjAcqThread->GetDeviceHandle(m_hDevice);

    // Get device info and show it on text label
    ShowDeviceInfo();

    // Get MainWindow param from device
    GetDeviceInitParam();

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Close Device
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::CloseDevice()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Stop Timer
    m_pobjShowImgTimer->stop();
    m_pobjShowFrameRateTimer->stop();

    // Stop acquisition thread before close device if acquisition did not stoped
    if (m_bAcquisitionStart)
    {
        m_pobjAcqThread->m_bAcquisitionThreadFlag = false;
        m_pobjAcqThread->quit();
        m_pobjAcqThread->wait();

        // isStart flag reset
        m_bAcquisitionStart = false;
    }

    // Release acquisition thread object
    RELEASE_ALLOC_MEM(m_pobjAcqThread);

    // Close Device
    emStatus = GXCloseDevice(m_hDevice);
    GX_VERIFY(emStatus);

    // isOpen flag reset
    m_bOpen = false;

    // release device handle
    m_hDevice = NULL;

    m_hProperty = NULL;

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Close Device(slot)
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_CloseDevice_clicked()
{
    // Clear Mainwindow items, especially clear ComboBox
    ClearUI();

    // Destroy dialogs
    DestroyDialogs();

    // Close Device
    CloseDevice();

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Get device info and show it on text label
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::ShowDeviceInfo()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    // 128 bytes for device info string is enough
    // Get Vendor Name

    GX_STRING_VALUE stStrVendorName;
    emStatus = GXGetStringValue(m_hDevice, "DeviceVendorName", &stStrVendorName);
    GX_VERIFY(emStatus);
    // Show device information on Info label
    ui->VendorName->setText(QString("VendorName: %1").arg(QString(stStrVendorName.strCurValue)));

    // Get Model Name
    GX_STRING_VALUE stStrModelName;
    emStatus = GXGetStringValue(m_hDevice, "DeviceModelName", &stStrModelName);
    GX_VERIFY(emStatus);
    // Show device information on Info label
    ui->ModelName->setText(QString("ModelName: %1").arg(QString(stStrModelName.strCurValue)));

    // Get Serial Number
    GX_STRING_VALUE stStrSerialNumber;
    emStatus = GXGetStringValue(m_hDevice, "DeviceSerialNumber", &stStrSerialNumber);
    // Show device information on Info label
    ui->SerialNumber->setText(QString("SN: %1").arg(QString(stStrSerialNumber.strCurValue)));

    // Get Device Version
    GX_STRING_VALUE stStrDeviceVersion;
    emStatus = GXGetStringValue(m_hDevice, "DeviceVersion", &stStrDeviceVersion);

    // Show device information on Info label
    ui->DeviceVersion->setText(QString("DeviceVersion: %1").arg(QString(stStrDeviceVersion.strCurValue)));

    return;
}

//----------------------------------------------------------------------------------
/**
\Set device acquisition buffer number.
\param[in]
\param[out]
\return bool    true : Setting success
\               false: Setting fail
*/
//----------------------------------------------------------------------------------
bool CGxViewer::SetAcquisitionBufferNum()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    uint64_t ui64BufferNum = 0;
    uint32_t i64PayloadSize = 0;

    // Get device current payload size
    uint32_t nDSNum = 0;
    emStatus = GXGetDataStreamNumFromDev(m_hDevice, &nDSNum);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
        return false;
    }

    if (nDSNum < 1)
    {
        printf("Failed to obtain the number of streams\n");
        return false;
    }

    GX_DS_HANDLE phDS = NULL;
    emStatus = GXGetDataStreamHandleFromDev(m_hDevice, 1, &phDS);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
        return false;
    }

    emStatus = GXGetPayLoadSize(phDS, &i64PayloadSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
        return false;
    }

    // Set buffer quantity of acquisition queue
    if (i64PayloadSize == 0)
    {
        QMessageBox::about(this, "Set Buffer Number", "Set acquisiton buffer number failed : Payload size is 0 !");
        return false;
    }

    // Calculate a reasonable number of Buffers for different payload size
    // Small ROI and high frame rate will requires more acquisition Buffer
    const size_t MAX_MEMORY_SIZE = 8 * 1024 * 1024; // The maximum number of memory bytes available for allocating frame Buffer
    const size_t MIN_BUFFER_NUM = 5;                // Minimum frame Buffer number
    const size_t MAX_BUFFER_NUM = 450;              // Maximum frame Buffer number
    ui64BufferNum = MAX_MEMORY_SIZE / i64PayloadSize;
    ui64BufferNum = (ui64BufferNum <= MIN_BUFFER_NUM) ? MIN_BUFFER_NUM : ui64BufferNum;
    ui64BufferNum = (ui64BufferNum >= MAX_BUFFER_NUM) ? MAX_BUFFER_NUM : ui64BufferNum;

    emStatus = GXSetAcqusitionBufferNumber(m_hDevice, ui64BufferNum);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
        return false;
    }

    // Transfer buffer number to acquisition thread class for using GXDQAllBufsEx
    m_pobjAcqThread->m_ui64AcquisitionBufferNum = ui64BufferNum;

    return true;
}

//----------------------------------------------------------------------------------
/**
\Set Gige device PacketSize and PacketDelay
\return bool    true : Setting success
\               false: Setting fail
*/
//----------------------------------------------------------------------------------
bool CGxViewer::SetPacketSizeAndPacketDelay()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    GX_NODE_ACCESS_MODE emAccessMode;

    // Set PacketSize
    emStatus = GXGetNodeAccessMode(m_hDevice, "GevSCPSPacketSize", &emAccessMode);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return false;
    }

    bool bImplementPacketSize = (emAccessMode == GX_NODE_ACCESS_MODE_RW) ? true : false;
    if (bImplementPacketSize)
    {
        uint32_t ui32PacketSize = 0;
        emStatus = GXGetOptimalPacketSize(m_hDevice, &ui32PacketSize);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            return false;
        }
        emStatus = GXSetIntValue(m_hDevice, "GevSCPSPacketSize", ui32PacketSize);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            return false;
        }
    }

    // Set PacketDelay
    emStatus = GXGetNodeAccessMode(m_hDevice, "GevSCPD", &emAccessMode);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return false;
    }

    bool bImplementPacketDelay = (emAccessMode == GX_NODE_ACCESS_MODE_RW) ? true : false;
    if (bImplementPacketDelay)
    {
        GX_INT_VALUE stValue;
        emStatus = GXGetIntValue(m_hDevice, "GevSCPD", &stValue);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            return false;
        }
        emStatus = GXSetIntValue(m_hDevice, "GevSCPD", stValue.nMin);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            return false;
        }
    }

    return true;
}

//----------------------------------------------------------------------------------
/**
\Get parameters from Device and set them into UI items
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::GetDeviceInitParam()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Disable all items to avoid user input while initializing
    DisableUI();

    // Device is a color-camera or not
    bool bColorFilter = false;
    GX_NODE_ACCESS_MODE emColorFilterAccessMode;
    emStatus = GXGetNodeAccessMode(m_hDevice, "PixelColorFilter", &emColorFilterAccessMode);

    bColorFilter = ((emColorFilterAccessMode == GX_NODE_ACCESS_MODE_WO) || (emColorFilterAccessMode == GX_NODE_ACCESS_MODE_RO) || (emColorFilterAccessMode == GX_NODE_ACCESS_MODE_RW)) ? true : false;

    if (emStatus != GX_STATUS_SUCCESS)
    {
        CloseDevice();
        GX_VERIFY(emStatus);
    }

    // Only color camera support image improvement
    if (bColorFilter)
    {
        // Alloc resource for image improvement, if allocate failed, disable ImageImprovement dialog
        bool bRet = false;
        bRet = m_pobjAcqThread->PrepareForImageImprovement();
        if (!bRet)
        {
            QMessageBox::about(NULL, "Error", "Prepare for image improvement Failed : Allocate LUT memory failed, image improvement disabled");
            ui->ImageImprovement->setEnabled(false);
        }
        else
        {
            ui->ImageImprovement->setEnabled(true);
        }
    }
    else
    {
        ui->ImageImprovement->setEnabled(false);
    }

    // Initialze success, enable all UI Items,
    EnableUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Check if MultiROI is opened(This sample does not support MultiROI)
\param[in]
\param[out]
\return bool        true       MultiROI is on or not implemented
                    false      MultiROI is off
*/
//----------------------------------------------------------------------------------
bool CGxViewer::CheckMultiROIOn()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    int64_t emRegionSendMode = GX_REGION_SEND_SINGLE_ROI_MODE;
    bool bRegionMode = false;
    GX_NODE_ACCESS_MODE emRegionMode;
    emStatus = GXGetNodeAccessMode(m_hDevice, "RegionSendMode", &emRegionMode);

    bRegionMode = ((emRegionMode == GX_NODE_ACCESS_MODE_WO) || (emRegionMode == GX_NODE_ACCESS_MODE_RO) || (emRegionMode == GX_NODE_ACCESS_MODE_RW)) ? true : false;

    if (emStatus != GX_STATUS_SUCCESS)
    {
        ShowErrorString(emStatus);
    }

    if (bRegionMode)
    {
        GX_ENUM_VALUE stValue;
        emStatus = GXGetEnumValue(m_hDevice, "RegionSendMode", &stValue);
        emRegionSendMode = stValue.stCurValue.nCurValue;
        if (emStatus != GX_STATUS_SUCCESS)
        {
            ShowErrorString(emStatus);
        }
    }

    if (emRegionSendMode == GX_REGION_SEND_MULTI_ROI_MODE)
    {
        QMessageBox::about(this, "MultiROI not supported", "This sample does not support MultiROI!\n"
                                                           "please change region mode!");
        return true;
    }

    return false;
}

//----------------------------------------------------------------------------------
/**
\Start Acquisition(slot)
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_StartAcquisition_clicked()
{
    bool bMultiRoiOn = false;
    // This sample does not support MultiROI, acquisition will not started when multiROI is on.
    bMultiRoiOn = CheckMultiROIOn();
    if (bMultiRoiOn)
    {
        return;
    }

    bool bSetDone = false;
    // Set acquisition buffer number
    bSetDone = SetAcquisitionBufferNum();
    if (!bSetDone)
    {
        return;
    }

    // Set packet size and packet delay
    bSetDone = SetPacketSizeAndPacketDelay();
    if (!bSetDone)
    {
        return;
    }

    bool bPrepareDone = false;
    // Alloc resource for image acquisition
    bPrepareDone = m_pobjAcqThread->PrepareForShowImg();
    if (!bPrepareDone)
    {
        return;
    }

    // Device start acquisition and start acquisition thread
    StartAcquisition();

    // Do not start timer when acquisition start failed
    if (!m_bAcquisitionStart)
    {
        return;
    }

    // Start image showing timer(Image show frame rate = 1000/nShowTimerInterval)
    // Refresh interval 33ms
    const int nShowTimerInterval = 33;
    m_pobjShowImgTimer->start(nShowTimerInterval);

    // Refresh interval 500ms
    const int nFrameRateTimerInterval = 500;
    m_pobjShowFrameRateTimer->start(nFrameRateTimerInterval);

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Device start acquisition and start acquisition thread
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::StartAcquisition()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    emStatus = GXStreamOn(m_hDevice);
    GX_VERIFY(emStatus);

    // Set acquisition thread run flag
    m_pobjAcqThread->m_bAcquisitionThreadFlag = true;

    // Acquisition thread start
    m_pobjAcqThread->start();

    // isStart flag set true
    m_bAcquisitionStart = true;

    return;
}

//----------------------------------------------------------------------------------
/**
\Stop Acquisition(slot)
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::on_StopAcquisition_clicked()
{
    // Stop Acquisition
    StopAcquisition();

    // Stop timer
    m_pobjShowImgTimer->stop();
    m_pobjShowFrameRateTimer->stop();

    m_objFps.Reset();

    // Reset acquisition frame rate to 0
    ui->AcqFrameRateLabel->setText(QString("Frame NUM: %1   Acq. FPS: %2")
                                       .arg(m_pobjAcqThread->m_nFrameCount)
                                       .arg(0.0, 0, 'f', 1));
    // Reset display frame rate
    ui->ShowFrameRateLabel->setText(QString("Disp. FPS: %1").arg(0.0, 0, 'f', 1));

    // Update Mainwindow
    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Device stop acquisition and stop acquisition thread
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::StopAcquisition()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    m_pobjAcqThread->m_bAcquisitionThreadFlag = false;
    m_pobjAcqThread->quit();
    m_pobjAcqThread->wait();

    // Turn off stream
    emStatus = GXStreamOff(m_hDevice);
    GX_VERIFY(emStatus);

    // isStart flag set false
    m_bAcquisitionStart = false;

    return;
}

//----------------------------------------------------------------------------------
/**
/Show images acquired and processed(slot)
\param[in]
\param[out]
\return void
*/
//----------------------------------------------------------------------------------
void CGxViewer::slotShowImage()
{
    // If acquisition did not started
    if (!m_bAcquisitionStart)
    {
        return;
    }

    // Get Image from image show queue, if image show queue is empty, return directly
    QImage *qobjImgShow = m_pobjAcqThread->PopFrontFromShowImageDeque();
    if (qobjImgShow == NULL)
    {
        return;
    }

    if (m_bSaveImage)
    {
        // Deep copy
        m_objImageForSave = qobjImgShow->copy();
        // Reset flag
        m_bSaveImage = false;
        // Emit a signal to save current image
        emit SigSaveImage();
    }

    // Display the image
    QImage objImgScaled = qobjImgShow->scaled(ui->ImageLabel->width(), ui->ImageLabel->height(),
                                              Qt::IgnoreAspectRatio, Qt::FastTransformation);
    ui->ImageLabel->setPixmap(QPixmap::fromImage(objImgScaled));

    // Display is finished, push back image buffer to buffer queue
    m_pobjAcqThread->PushBackToEmptyBufferDeque(qobjImgShow);

    // Calculate image showing frame rate
    m_ui32ShowCount++;

    m_objFps.IncreaseFrameNum();

    return;
}

//----------------------------------------------------------------------------------
/**
\Get image show frame rate
\param[in]
\param[out]
\return  double
*/
//----------------------------------------------------------------------------------
double CGxViewer::GetImageShowFps()
{
    double dImgShowFrameRate = 0.0;

    m_objFps.UpdateFps();
    dImgShowFrameRate = m_objFps.GetFps();

    return dImgShowFrameRate;
}

//----------------------------------------------------------------------------------
/**
\Get acquisition frame count and frame rate from acquisition thread(slot)
\param[in]
\param[in]
\param[out]
\return  void
*/
//----------------------------------------------------------------------------------
void CGxViewer::slotShowFrameRate()
{
    double dImgShowFrameRate = GetImageShowFps();
    double dAcqFrameRate = m_pobjAcqThread->GetImageAcqFps();

    // Show acquisition frame count and frame rate
    ui->AcqFrameRateLabel->setText(QString("Frame NUM: %1   Acq. FPS: %2")
                                       .arg(m_pobjAcqThread->m_nFrameCount)
                                       .arg(dAcqFrameRate, 0, 'f', 1));
    // Show display frame rate
    ui->ShowFrameRateLabel->setText(QString("Disp.FPS: %1").arg(dImgShowFrameRate, 0, 'f', 1));

    return;
}

//----------------------------------------------------------------------------------
/**
\Refresh Main window when execute usersetload
\param[in]
\param[out]
\return  void
*/
//----------------------------------------------------------------------------------
void CGxViewer::slotRefreshMainWindow()
{
    this->GetDeviceInitParam();

    return;
}

//----------------------------------------------------------------------------------
/**
\Get error from acquisition thread and show error message
\param[in]      szErrorString  Error string from acquisition thread
\param[out]
\return  void
*/
//----------------------------------------------------------------------------------
void CGxViewer::slotAcquisitionErrorHandler(QString szErrorString)
{
    // Show error and stop acquisition
    QMessageBox::about(NULL, "Acquisition Error", szErrorString);

    StopAcquisition();

    UpdateUI();

    return;
}

//----------------------------------------------------------------------------------
/**
\Get error from image processing and show error message
\param[in]     emStatus     Error code from image processing
\param[out]
\return  void
*/
//----------------------------------------------------------------------------------
void CGxViewer::slotImageProcErrorHandler(VxInt32 emStatus)
{
    // Show error and stop acquisition : Error code 0 means pixel format not support
    if(emStatus == 0)
    {
        QMessageBox::about(NULL, "Image Process Error", QString("Error : Unsupported pixel format!"));
    }
    else
    {
        QMessageBox::about(NULL, "Image Process Error", QString("Error : Image Processing Failed, Error code : %1").arg(emStatus));
    }
    StopAcquisition();

    UpdateUI();

    return;
}

void CGxViewer::on_bt_OpenProperties_clicked()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    if (NULL == m_hProperty)
    {
        return;
    }

    QPoint pos = this->frameGeometry().topLeft();
    int32_t i32Width = this->frameGeometry().width();
    int32_t i32Height = this->frameGeometry().height();
    QPoint propertyPos = QPoint(pos.x() + i32Width, pos.y());
    emStatus = GXSetShowPosition(m_hProperty, propertyPos.x() + 2, propertyPos.y() + 4, DEFAULT_PROP_WIDTH, std::max(i32Height, MIN_PROP_HEIGHT));
    GX_VERIFY(emStatus);
    emStatus = GXShowWnd(m_hProperty, true);
    GX_VERIFY(emStatus);
}
