
# Bluetooth A2DP properties

PRODUCT_PRODUCT_PROPERTIES += persist.bluetooth.bluetooth_audio_hal.disabled=true
PRODUCT_PRODUCT_PROPERTIES += persist.bt.a2dp.codec.support=3

MB_BT_SCRIPT := $(PWD)/vendor/mercedes/packages/MBBluetooth/mb_bt.sh
$(shell chmod 777 $(MB_BT_SCRIPT))
$(shell $(MB_BT_SCRIPT) $(PWD))
