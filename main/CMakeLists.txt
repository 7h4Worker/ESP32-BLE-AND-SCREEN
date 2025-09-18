# 使用最终优化版本
set(MAIN_SOURCES 
    gattc_demo_final.c
    GenJyuuGothic_32.c
)

# LVGL demo 源文件目录（根据实际路径调整）
set(LV_DEMO_DIR ../managed_components/lvgl__lvgl/demos)
file(GLOB_RECURSE LV_DEMOS_SOURCES ${LV_DEMO_DIR}/*.c)

# 注册 main 组件
idf_component_register(
    SRCS ${MAIN_SOURCES} ${LV_DEMOS_SOURCES}
    INCLUDE_DIRS . ${LV_DEMO_DIR}
    REQUIRES lvgl
)

# 获取 LVGL 库对象
idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)

# 给 LVGL 库添加宏定义
target_compile_options(
    ${LVGL_LIB}
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
        -DLV_USE_DEMO_MUSIC
)
