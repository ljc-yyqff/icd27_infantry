#!/bin/bash
# ***************************************************************************************
#
#
# ***************************************************************************************

DISTRIBUTION="unknown"

if [ -f /etc/lsb-release ]; then
    if [ -f /lib/lsb/init-functions ]; then
        DISTRIBUTION=`grep DISTRIB_ID /etc/lsb-release | sed -e 's/DISTRIB_ID=//'`
    fi
elif [ -f /etc/os-release ]; then
    if [ -f /lib/lsb/init-functions ]; then
        DISTRIBUTION=`grep -w NAME /etc/os-release | sed 's/[^"]*"\([^"]*\)"/\1/'`
        if [ "$DISTRIBUTION" == "Debian GNU/Linux" ] ; then
            # Debian is close enough for the purposes of this script
            DISTRIBUTION="Ubuntu"
        fi
    fi
fi

# do as sudo or root
if [ $(id -u) -ne 0 ] ; then
    echo "User has insufficient privileges. Please try sudo command."
    exit 1
fi

GRUB_FILE="/etc/default/grub"
PARAMS="usbcore.usbfs_memory_mb=2000 swiotlb=131072"

tmpfile=$(mktemp)
found=0   # 用于记录是否找到 GRUB_CMDLINE_LINUX 行

while IFS= read -r line; do
    # 只处理 GRUB_CMDLINE_LINUX 行
    if [[ "$line" =~ ^GRUB_CMDLINE_LINUX= ]]; then
        found=1
        # 提取原有内容
        current=$(echo "$line" | sed 's/^GRUB_CMDLINE_LINUX=//; s/"//g')
        new="$current"

        for p in $PARAMS; do
            key=${p%=*}
            val=${p#*=}

            if echo "$new" | grep -Eq "(^|[[:space:]])${key}="; then
                new=$(echo "$new" | sed -E "s#(^|[[:space:]])${key}=[^[:space:]]*#\\1${key}=${val}#g")
            else
                # 追加参数
                new="$new $key=$val"
            fi
        done

        # 去掉前后空格
        new=$(echo "$new" | sed 's/^ *//;s/ *$//')

        echo "GRUB_CMDLINE_LINUX=\"$new\"" >> "$tmpfile"
    else
        # 非目标行保持不变
        echo "$line" >> "$tmpfile"
    fi
done < "$GRUB_FILE"

# 如果没有找到 GRUB_CMDLINE_LINUX 行，则添加一行
if [ $found -eq 0 ]; then
    new=""
    for p in $PARAMS; do
        key=${p%=*}
        val=${p#*=}
        new="$new $key=$val"
    done
    new=$(echo "$new" | sed 's/^ *//;s/ *$//')
    echo "GRUB_CMDLINE_LINUX=\"$new\"" >> "$tmpfile"
fi

sudo cp "$tmpfile" "$GRUB_FILE"
rm "$tmpfile"

echo "Updated GRUB_CMDLINE_LINUX."

if [ "$DISTRIBUTION" = "Ubuntu" ]; then
    update-grub
else
    grub2-mkconfig -o /boot/grub2/grub.cfg
fi

