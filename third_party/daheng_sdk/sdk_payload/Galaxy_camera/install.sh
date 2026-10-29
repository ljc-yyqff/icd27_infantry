#!/bin/bash
#-------------------------------------------------------------
#**
#file      install.sh
#brief     Galaxy embedded SDK install script 
#version   1.0.2110.8201
#date      2025.02.17
#/
#-------------------------------------------------------------

# Return 0 for 'n', 1 for 'y'
ChooseYorN() 
{
    local q="$1 (Y/n) : "
    while true; do
        read -p "$q" yn
        case $yn in
            [Yy]* ) return 0; ;;
            [Nn]* ) return 1; ;;
            "" ) return 0; ;;
            * );;
        esac
    done
}

#returns 0 when the user answered en
#returns 1 when the user answered cn
askLanguage() {
        local q="$1 (En/Cn) : "
        while true; do
            read -p "$q" language
            case $language in 
                [Ee]* ) return 1; ;;
                [Cc]* ) return 0; ;;
                "" ) return 1; ;;
                * );;
            esac
        done
}

#return 1 for '32', 0 for '64'
ChooseSDK() {
        local q="$1 (32/64) : "
        while true; do
            read -p "$q" sdk
            case $sdk in 
                32 ) return 1; ;;
                64 ) return 0; ;;
                "" ) return 0; ;;
                * );;
            esac
        done
}

SUDO=$1
CURRENT_PATH=`pwd`
ARCH_1="x86_64"
ARCH_2="x86"

ARCH=$ARCH_1

if command -v getconf >/dev/null 2>&1; then
    if [ $(getconf LONG_BIT) == '32' ];
    then 
        echo "32-bit systems are not supported."
        exit 1
    fi
fi

if [ $ARCH == $ARCH_1 ] ;
then 
    BIN_BIT=x64
else
    BIN_BIT=x32
fi 
ls | grep bin | grep -v $BIN_BIT | xargs rm -rf
mv bin_$BIN_BIT bin

LIB_PATH=$CURRENT_PATH/lib/$ARCH
cd "$CURRENT_PATH"/lib 
ls | grep -v -w $ARCH | xargs rm -rf
echo ""

INSTRUCTION_CN="C软件开发说明书.pdf"
SOFTUSER_CN="软件使用说明书.pdf"
INSTRUCTION_EN=""
FAQ_CN="Linux版SDK常见问题解答.pdf"
FAQ_EN=""
DOTNET_DOC_CN="DotNET软件开发说明书.pdf"
DOTNET_DOC_EN="DotNET SDK Programming Reference Manual.pdf"
FAQ_DOC_EN="Linux SDK FAQs.pdf"
#returns 0 when the user answered en
#returns 1 when the user answered cn

if [ ! -d "/etc/Galaxy/cfg/viewer/" ] ;
then
    $SUDO mkdir -p "/etc/Galaxy/cfg/viewer/"
fi
if askLanguage "Choose language?" ; 
then
    rm -f "$CURRENT_PATH"/Development/Doc/C\ SDK\ Programming\ Reference\ Manual.pdf
	rm -f "$CURRENT_PATH"/Development/Doc/"$FAQ_DOC_EN"
	rm -f "$CURRENT_PATH"/Development/Doc/User\ Manual.pdf
    rm -f "$CURRENT_PATH"/Development/Doc/"$DOTNET_DOC_EN"
    rm -f "$CURRENT_PATH"/bin/resource/settings/GalaxyView.ini
    mv -f "$CURRENT_PATH"/bin/resource/settings/GalaxyView_cn.ini "/etc/Galaxy/cfg/viewer/GalaxyView.ini"
    rm -f "$CURRENT_PATH"/bin/resource/settings/GalaxyView_cn.ini
else
    rm -f "$CURRENT_PATH"/Development/Doc/"$INSTRUCTION_CN"
	rm -f "$CURRENT_PATH"/Development/Doc/"$FAQ_CN"
    rm -f "$CURRENT_PATH"/Development/Doc/"$DOTNET_DOC_CN"
    rm -f "$CURRENT_PATH"/Development/Doc/"$SOFTUSER_CN"
    mv -f "$CURRENT_PATH"/bin/resource/settings/GalaxyView.ini "/etc/Galaxy/cfg/viewer/GalaxyView.ini"
    rm -f "$CURRENT_PATH"/bin/resource/settings/GalaxyView.ini
fi
rm -rf "$CURRENT_PATH"/bin/resource/settings


USR_LIB=/usr/lib
LIB_GXIAPI=libgxiapi.so
LIB_GXU3VTL=GxU3VTL.cti
LIB_GXGVTL=GxGVTL.cti
LIB_NET6=net6.0
CONFIG_PATH="$CURRENT_PATH"/config

# Exit if user lib did not exist
if [ ! -d "$USR_LIB" ] ; 
then
    echo "***Libary file add failed*** : Libary directory '$USR_LIB' not found in your system."
    echo "Script Exit..."
    exit 1;
fi
# Exit if lib not found in package
if [ ! -e "$LIB_PATH/$LIB_GXIAPI" ] || [ ! -e "$LIB_PATH/$LIB_GXU3VTL" ] || [ ! -e "$LIB_PATH/$LIB_GXGVTL" ]; 
then
    echo "***Libary file add failed*** : Libary missing!"
    echo "Script Exit..."
    exit 1;
fi

# Override old library or not
if [ -e "$USR_LIB/$LIB_GXIAPI" ] || [ -e "$USR_LIB/$LIB_GXU3VTL" ] || [ -e "$USR_LIB/$LIB_GXGVTL" ] || [ ! -e "$LIB_PATH/$LIB_NET6" ] || [ -L "$USR_LIB/$LIB_GXIAPI" ] || [ -L "$USR_LIB/$LIB_GXU3VTL" ] || [ -L "$USR_LIB/$LIB_GXGVTL" ]; 
then      
    if ChooseYorN "$LIB_GXIAPI or $LIB_GXU3VTL or $LIB_GXGVTL or $LIB_NET6 already exist, are you sure to override?"; 
    then
    	$SUDO cp -a "$LIB_PATH"/* $USR_LIB
    	echo "Libary already override."
    	echo ""
    	else
    	echo "Libary did not override."
    	echo ""
    fi
else
    $SUDO cp -a "$LIB_PATH"/* $USR_LIB
fi

# Copy udev file to get device access permission of normal user, take effect after replug.
UDEV_RULES_PATH=/etc/udev/rules.d
UDEV_RULES_FILE=99-galaxy-dev.rules
if [ -e "$CONFIG_PATH"/$UDEV_RULES_FILE ] ;
then
    if [ -d "$UDEV_RULES_PATH" ] ; 
    then
       $SUDO cp -pdrf "$CONFIG_PATH"/$UDEV_RULES_FILE $UDEV_RULES_PATH
	   $SUDO service udev reload &> /dev/null
	   $SUDO service udev restart &> /dev/null
	   $SUDO systemctl restart systemd-udevd.service &> /dev/null
    else
        echo "***Udev file add failed*** : Udev rules directory '$UDEV_RULES_PATH' not found in your system.
You may need to add your device access permission manually,
or you can use 'sudo' to get permission to access device." 
        echo "" 
    fi
else
    echo "***Udev file add failed*** : Udev rules file '$UDEV_RULES_FILE' missing!.
You may need to add your device access permission manually,
or you can use 'sudo' to get permission to access device." 
    echo "" 
fi

# Copy limits file to get increase thread priority permission, not particularly necessary, take effect after reboot.
LIMITS_CONF_PATH=/etc/security/limits.d
LIMITS_CONF_FILE=galaxy-limits.conf
if [ -e "$CONFIG_PATH"/$LIMITS_CONF_FILE ] ;
then
    if [ -d "$LIMITS_CONF_PATH" ] ; 
    then
        $SUDO cp -pdrf "$CONFIG_PATH"/$LIMITS_CONF_FILE $LIMITS_CONF_PATH
    else
        echo "***Limits file add failed*** : Limits conf directory '$LIMITS_CONF_PATH' not found in your system."
        echo ""
    fi
else
    echo "***Limits file add failed*** : Limits conf file '$LIMITS_CONF_FILE' missing!." 
    echo ""
fi

EXTLINUX_CONF=/boot/extlinux/extlinux.conf
USB_MEM_MB=/sys/module/usbcore/parameters/usbfs_memory_mb
if [ -f $USB_MEM_MB ]; then
    $SUDO echo 2000 > $USB_MEM_MB
fi

if command -v update-grub >/dev/null 2>&1; then
    $SUDO bash "$CURRENT_PATH"/set_usbfs_memory_size.sh
elif [ -f $EXTLINUX_CONF ]; then
    if grep -E 'usbcore\.usbfs_memory_mb=' $EXTLINUX_CONF > /dev/null 2>&1; then
        $SUDO sed -i 's/usbcore\.usbfs_memory_mb=[0-9]*/usbcore.usbfs_memory_mb=2000/g' $EXTLINUX_CONF
    else
        $SUDO sed -i 's/APPEND/APPEND usbcore.usbfs_memory_mb=2000/g' $EXTLINUX_CONF
    fi

    if grep swiotlb $EXTLINUX_CONF > /dev/null; then
        echo "" > /dev/null
    else
        $SUDO sed -i 's/APPEND/APPEND swiotlb=131072/g' $EXTLINUX_CONF
    fi
fi

echo "copy config file to cfg"
if [ ! -d "/etc/Galaxy/cfg" ] ;
then
    $SUDO mkdir -p "/etc/Galaxy/cfg"
fi

$SUDO cp -f "$CURRENT_PATH"/config/log4cplus.properties "/etc/Galaxy/cfg/"
$SUDO chmod 666 "/etc/Galaxy/cfg/log4cplus.properties"

#echo "copy config ToolsConfig.ini file to cfg"
if [ ! -d "/etc/Galaxy/cfg/tools/" ] ;
then
    $SUDO mkdir -p "/etc/Galaxy/cfg/tools/"
fi
$SUDO cp -f "$CURRENT_PATH"/config/ToolsConfig.ini "/etc/Galaxy/cfg/tools/"

#echo "copy viewer config file to cfg"
if [ ! -d "/etc/Galaxy/cfg/viewer/" ] ;
then
    $SUDO mkdir -p "/etc/Galaxy/cfg/viewer/"
fi
$SUDO mv -f "$CURRENT_PATH"/bin/resource/gxplugins/Plugin.ini "/etc/Galaxy/cfg/viewer/Plugin.ini"

#echo "copy PluginConfig config file to cfg"
if [ ! -d "/etc/Galaxy/cfg/gxplugins/" ] ;
then
    $SUDO mkdir -p "/etc/Galaxy/cfg/gxplugins/"
fi
$SUDO mv -f "$CURRENT_PATH"/bin/resource/gxplugins/PluginConfig.ini "/etc/Galaxy/cfg/gxplugins/PluginConfig.ini"

#echo "copyfont file to Galaxy"
$SUDO cp -rf "$CURRENT_PATH"/config/font "/etc/Galaxy/"
if [ -d "/etc/profile.d" ] ;
then
    $SUDO cp -f "$CURRENT_PATH"/config/galaxy-env.sh "/etc/profile.d/"
fi

if [ ! -d "/var/log/Galaxy" ] ;
then
    echo "var log dir not exit will create"
    $SUDO mkdir -p "/var/log/Galaxy"
    $SUDO touch "/var/log/Galaxy/SDK.log"
    $SUDO chmod 777 "/var/log/Galaxy"
    $SUDO chmod 666 /var/log/Galaxy/*
else
    $SUDO chmod 777 "/var/log/Galaxy"
    $SUDO chmod 666 /var/log/Galaxy/*
fi

#copy cxp
if [ -d "$CURRENT_PATH"/Galaxy/CXP ]
then
    $SUDO cp -pdrf "$CURRENT_PATH"/Galaxy/CXP /etc/Galaxy
fi

if [ -d "$CURRENT_PATH"/Galaxy/GEV ]
then
    $SUDO cp -pdrf "$CURRENT_PATH"/Galaxy/GEV /etc/Galaxy
fi

#copy galaxyview config
$SUDO cp -f "$CURRENT_PATH"/ReleaseNote.txt "/etc/Galaxy/"
$SUDO cp -rf "$CURRENT_PATH"/bin/resource/translation "/etc/Galaxy/"

if [ ! -d "/etc/Galaxy/userdata" ]
then
    echo "userdata dir not exit will create"
    $SUDO mkdir -p "/etc/Galaxy/userdata"
fi

$SUDO chmod -R 777 "/etc/Galaxy"

#install driver
DRIVER_PATH=/lib/modules/$(uname -r)/kernel/drivers/galaxy
if [ ! -d $DRIVER_PATH ]
then
    $SUDO mkdir -p "$DRIVER_PATH"
fi

if lsmod | grep -q '^GxGEVGrabber'; then
    sudo modprobe -r GxGEVGrabber
fi
if lsmod | grep -q '^GxCXPGrabber'; then
    sudo modprobe -r GxCXPGrabber
fi

if lsmod | grep -q '^GxMemManager'; then
    sudo modprobe -r GxMemManager
fi

if lsmod | grep -q '^GxU3VBase'; then
    sudo modprobe -r GxU3VBase
fi

# 删除之前安装包残留的 GxMemManager 驱动
$SUDO rm -f "$DRIVER_PATH"/GxMemManager.ko

cd "$CURRENT_PATH"/driver/GxCXPGrabber
make
$SUDO cp -f GxCXPGrabber.ko "$DRIVER_PATH"

cd $CURRENT_PATH/driver/GxGEVGrabber
make
$SUDO cp -f GxGEVGrabber.ko $DRIVER_PATH

cd $CURRENT_PATH/driver/GxU3VBase
make
$SUDO cp -f GxU3VBase.ko $DRIVER_PATH

$SUDO depmod -a
$SUDO modprobe GxCXPGrabber
$SUDO echo GxCXPGrabber > /etc/modules-load.d/galaxy_driver.conf
$SUDO modprobe GxGEVGrabber
$SUDO echo GxGEVGrabber >> /etc/modules-load.d/galaxy_driver.conf
$SUDO modprobe GxU3VBase
$SUDO echo GxU3VBase >> /etc/modules-load.d/galaxy_driver.conf

$SUDO setcap cap_net_raw+ep $CURRENT_PATH/bin/GxGigeIPConfig

cd "$CURRENT_PATH"
rm set_usbfs_memory_size.sh

# 32bit linux do not support CSharp 
if [ $BIN_BIT == x32 ];
then
    rm -rf sample/CSharp 
fi

$SUDO $CURRENT_PATH/enable-cpu-performance.sh
$SUDO $CURRENT_PATH/set_all_nic_mtu_9000.sh
$SUDO $CURRENT_PATH/SetSocketBufferSize.sh

echo "-----------------------------------------------------------------"
echo "All configurations will take effect after the system is rebooted"
echo "If you don't want to reboot the system for a while"
echo "you will need to unplug and replug the camera."
echo "-----------------------------------------------------------------"
