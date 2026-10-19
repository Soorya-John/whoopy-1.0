idf_component_register(SRCS "app_main.c" "hid_input.c" "radio_tx.c"
 INCLUDE_DIRS "." "../../../shared/protocol"
 REQUIRES driver usb usb_host_hid)
