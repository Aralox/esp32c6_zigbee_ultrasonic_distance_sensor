idf_component_register(
    SRCS "tank.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp-zboss-lib
        esp-zigbee-lib
        nvs_flash
)

# make sure CONFIG_ZB_ENABLED is enabled in SDK configuration.
# https://stackoverflow.com/questions/74900268/esp-idf-build-fails-with-fatal-error-esp-wifi-h-no-such-file-or-directory?rq=1
# https://stackoverflow.com/questions/77234505/how-to-include-things-from-esp-idf
# Similar to \esp-zigbee-sdk-main\components\esp-zigbee-ncp\CMakeLists.txt