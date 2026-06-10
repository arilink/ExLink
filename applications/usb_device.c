/*
 * Copyright (c) 2026 LabForge / ExLink
 * Portions derived from CherryUSB templates (Apache-2.0, sakumisu):
 *   winusb2.0_cdc_template.c, cdc_acm_multi_template.c, hid_custom_inout_template.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ============================================================================
 *  ExLink USB 复合设备 (F1) —— 描述符 + 装配核心
 * ============================================================================
 *
 *  把 M1 的单接口 vendor 设备长成 ARCHITECTURE.md §6.3 冻结的 7 接口复合设备:
 *
 *    设备级: bDeviceClass = 0xEF/0x02/0x01 (IAD/复合), bcdDevice 0x0102
 *            (M1 单功能是 0x00/.../0x0101; 这里变复合 + bump 版本, 让 Windows
 *             当新硬件重枚举, 避开 M1 单功能驱动节点缓存)。
 *
 *    MS OS 2.0: SET_HEADER + 2 个 FUNCTION_SUBSET (仅 IF0 命令 / IF1 DAP-v2 两个
 *               vendor 接口绑 WinUSB; CDC 走 usbser/cdc_acm, HID 走 HID 类, 不进集)。
 *               —— 这正是 M1 注释里"变真复合时切回 function subset"的落地。
 *
 *  接口/端点表与各功能模块的对口见 usb_device.h。 本文件只拥有**静态描述符字节**
 *  和**装配编排** (exlink_usb_init); 各接口的运行时端点逻辑在:
 *    IF0          -> usb_if_ctrl.c    (控制通道/命令帧, M1 行为不变)
 *    IF1          -> usb_if_dap.c     (CMSIS-DAP v2 bulk, F1 仅枚举桩, 真逻辑 F3)
 *    IF2          -> usb_if_dapv1.c   (CMSIS-DAP v1 HID, F1 仅枚举桩, 真逻辑 F3)
 *    IF3..IF6     -> usb_if_uart.c    (2x CDC ACM, F1 回环 echo, 真 UART 桥接 F4)
 * ============================================================================
 */
#include <rtthread.h>

#include "usbd_core.h"
#include "usbd_cdc_acm.h"   /* CDC_ACM_DESCRIPTOR_INIT / CDC_ACM_DESCRIPTOR_LEN */
#include "usbd_hid.h"       /* HID_DESCRIPTOR_TYPE_HID */

#include "usb_device.h"
#include "frame_proto.h"    /* EXLINK_USB_VID / EXLINK_USB_PID (跨端契约, 单一真相源) */
#include "usb_if_ctrl.h"
#include "usb_if_dap.h"
#include "usb_if_dapv1.h"
#include "usb_if_uart.h"

#define USBD_MAX_POWER      200
#define USBD_LANGID_STRING  1033

/* ---- vendor control 请求码 (MS OS 2.0 取描述符用) ---- */
#define EXLINK_WINUSB_VENDOR_CODE   0x20

/* bulk 端点包长 (HS/FS 各一份描述符) */
#define EXLINK_BULK_MPS_HS  512u
#define EXLINK_BULK_MPS_FS  64u

/* config 描述符总长 (用宏拼, 不手填):
 *   config(9)
 *   + IF0 vendor: intf(9)+ep(7)+ep(7)
 *   + IF1 vendor: intf(9)+ep(7)+ep(7)
 *   + IF2 HID   : intf(9)+hid(9)+ep(7)+ep(7)
 *   + CDC1 / CDC2: 各 CDC_ACM_DESCRIPTOR_LEN (含 IAD)
 */
#define USB_CONFIG_SIZE (9 + (9 + 7 + 7) + (9 + 7 + 7) + (9 + 9 + 7 + 7) \
                         + CDC_ACM_DESCRIPTOR_LEN + CDC_ACM_DESCRIPTOR_LEN)

/* ===========================================================================
 *  设备描述符 (复合 / IAD)
 * ========================================================================= */
static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0xEF, 0x02, 0x01,
                               EXLINK_USB_VID, EXLINK_USB_PID, 0x0102, 0x01)
};

/* ===========================================================================
 *  config 描述符 (双速; 用一个宏展开 config/other-speed × HS/FS 共 4 份)
 *
 *  注: USB_CONFIG_DESCRIPTOR_INIT 把 bDescriptorType 写死 0x02, 无法用于 other-speed
 *  (0x07), 故手拼 9 字节 config 头 (沿用 M1 做法), 仅 cfg_type / bulk_mps 变化。
 * ========================================================================= */
/* iInterface 字符串索引 (实体见下方 string_descriptors[]); 给两个 vendor 接口各自
 * 命名, 否则 Windows 对两个 WinUSB 接口都退回显示 Product 名 (都叫 "ExLink"), 难区分。 */
#define EXLINK_ISTR_CMD     4u   /* IF0: "ExLink Command" */
#define EXLINK_ISTR_DAP     5u   /* IF1: "ExLink CMSIS-DAP v2" */
#define EXLINK_ISTR_DAPV1   6u   /* IF2: "ExLink CMSIS-DAP v1" */

#define EXLINK_CONFIG_BLOB(cfg_type, bulk_mps)                                                       \
    /* configuration header */                                                                       \
    0x09, (cfg_type), WBVAL(USB_CONFIG_SIZE), EXLINK_IFNUM_TOTAL, 0x01, 0x00,                         \
    USB_CONFIG_BUS_POWERED, USB_CONFIG_POWER_MA(USBD_MAX_POWER),                                      \
    /* ---- IF0: vendor bulk 命令通道 (WinUSB) ---- */                                               \
    USB_INTERFACE_DESCRIPTOR_INIT(EXLINK_IFNUM_CMD, 0x00, 0x02, 0xFF, 0x00, 0x00, EXLINK_ISTR_CMD),   \
    USB_ENDPOINT_DESCRIPTOR_INIT(EXLINK_EP_CMD_OUT, USB_ENDPOINT_TYPE_BULK, (bulk_mps), 0x00),        \
    USB_ENDPOINT_DESCRIPTOR_INIT(EXLINK_EP_CMD_IN,  USB_ENDPOINT_TYPE_BULK, (bulk_mps), 0x00),        \
    /* ---- IF1: vendor bulk CMSIS-DAP v2 (WinUSB) ---- */                                           \
    USB_INTERFACE_DESCRIPTOR_INIT(EXLINK_IFNUM_DAP, 0x00, 0x02, 0xFF, 0x00, 0x00, EXLINK_ISTR_DAP),   \
    USB_ENDPOINT_DESCRIPTOR_INIT(EXLINK_EP_DAP_OUT, USB_ENDPOINT_TYPE_BULK, (bulk_mps), 0x00),        \
    USB_ENDPOINT_DESCRIPTOR_INIT(EXLINK_EP_DAP_IN,  USB_ENDPOINT_TYPE_BULK, (bulk_mps), 0x00),        \
    /* ---- IF2: HID CMSIS-DAP v1 (interrupt IN/OUT 64) ---- */                                      \
    USB_INTERFACE_DESCRIPTOR_INIT(EXLINK_IFNUM_HID, 0x00, 0x02, 0x03, 0x00, 0x00, EXLINK_ISTR_DAPV1),  \
    0x09, HID_DESCRIPTOR_TYPE_HID, 0x11, 0x01, 0x00, 0x01, 0x22, WBVAL(EXLINK_HID_REPORT_DESC_SIZE),  \
    USB_ENDPOINT_DESCRIPTOR_INIT(EXLINK_EP_HID_IN,  USB_ENDPOINT_TYPE_INTERRUPT, EXLINK_HID_MPS, 0x01),\
    USB_ENDPOINT_DESCRIPTOR_INIT(EXLINK_EP_HID_OUT, USB_ENDPOINT_TYPE_INTERRUPT, EXLINK_HID_MPS, 0x01),\
    /* ---- IF3+IF4: CDC1 ACM (UART1 透传), 宏内自带 IAD + notify(8B) + 数据 bulk ---- */             \
    CDC_ACM_DESCRIPTOR_INIT(EXLINK_IFNUM_CDC1, EXLINK_EP_CDC1_INT, EXLINK_EP_CDC1_OUT,                \
                            EXLINK_EP_CDC1_IN, (bulk_mps), 0x00),                                     \
    /* ---- IF5+IF6: CDC2 ACM (UART2 透传) ---- */                                                    \
    CDC_ACM_DESCRIPTOR_INIT(EXLINK_IFNUM_CDC2, EXLINK_EP_CDC2_INT, EXLINK_EP_CDC2_OUT,                \
                            EXLINK_EP_CDC2_IN, (bulk_mps), 0x00)

static const uint8_t config_descriptor_hs[] = {
    EXLINK_CONFIG_BLOB(USB_DESCRIPTOR_TYPE_CONFIGURATION, EXLINK_BULK_MPS_HS)
};
static const uint8_t config_descriptor_fs[] = {
    EXLINK_CONFIG_BLOB(USB_DESCRIPTOR_TYPE_CONFIGURATION, EXLINK_BULK_MPS_FS)
};
/* other-speed config (0x07): 端点取"另一速度"的包长 */
static const uint8_t other_speed_descriptor_hs[] = {   /* 当前 FS 时上报: HS 端点 (512) */
    EXLINK_CONFIG_BLOB(USB_DESCRIPTOR_TYPE_OTHER_SPEED, EXLINK_BULK_MPS_HS)
};
static const uint8_t other_speed_descriptor_fs[] = {   /* 当前 HS 时上报: FS 端点 (64) */
    EXLINK_CONFIG_BLOB(USB_DESCRIPTOR_TYPE_OTHER_SPEED, EXLINK_BULK_MPS_FS)
};

static const uint8_t device_quality_descriptor[] = {
    0x0a,                                   /* bLength = 10 */
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,   /* 0x06 */
    0x10, 0x02,                             /* bcdUSB 0x0210 */
    0xEF,                                   /* bDeviceClass (复合) */
    0x02,                                   /* bDeviceSubClass */
    0x01,                                   /* bDeviceProtocol */
    0x40,                                   /* bMaxPacketSize0 = 64 (其他速度下 EP0) */
    0x01,                                   /* bNumConfigurations = 1 */
    0x00,                                   /* bReserved */
};

/* ===========================================================================
 *  MS OS 2.0 descriptor set —— 复合设备: SET_HEADER + 2 个 FUNCTION_SUBSET
 *  仅 IF0 (命令) / IF1 (DAP-v2) 两个 vendor 接口绑 WinUSB, 各用独立 GUID。
 * ========================================================================= */
#define WINUSB_DESCRIPTOR_SET_HEADER_SIZE  10
#define WINUSB_FUNCTION_SUBSET_HEADER_SIZE 8
#define WINUSB_FEATURE_COMPATIBLE_ID_SIZE  20
#define DEVICE_INTERFACE_GUIDS_FEATURE_LEN 132
#define EXLINK_FUNC_SUBSET_LEN  (WINUSB_FUNCTION_SUBSET_HEADER_SIZE + \
                                 WINUSB_FEATURE_COMPATIBLE_ID_SIZE + \
                                 DEVICE_INTERFACE_GUIDS_FEATURE_LEN)   /* 8+20+132 = 160 */
#define USBD_WINUSB_DESC_SET_LEN (WINUSB_DESCRIPTOR_SET_HEADER_SIZE + \
                                  2u * EXLINK_FUNC_SUBSET_LEN)         /* 10+160+160 = 330 */

/* 一个 function subset: bFirstInterface + WINUSB compat id + DeviceInterfaceGUID。
 * GUID = {7D9A0E11-4C4B-4E6C-9B3A-7D4C4B4E6C0X}, X = _last ('1'=命令, '2'=DAP-v2)。
 * (随机生成的私有 GUID, 上位机 libusb 不依赖它, 仅供 Windows WinUSB 绑定。)
 */
#define EXLINK_WINUSB_FUNC_SUBSET(_ifnum, _last)                       \
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE),                         \
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE),                         \
    (_ifnum), 0x00,                                                    \
    WBVAL(EXLINK_FUNC_SUBSET_LEN),                                     \
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),                          \
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),                          \
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,                                \
    0, 0, 0, 0, 0, 0, 0, 0,                                            \
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN),                         \
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),                           \
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ),                         \
    WBVAL(42),                                                         \
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,                    \
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0, \
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,                      \
    WBVAL(80),                                                         \
    '{', 0,                                                            \
    '7', 0, 'D', 0, '9', 0, 'A', 0, '0', 0, 'E', 0, '1', 0, '1', 0, '-', 0, \
    '4', 0, 'C', 0, '4', 0, 'B', 0, '-', 0,                            \
    '4', 0, 'E', 0, '6', 0, 'C', 0, '-', 0,                            \
    '9', 0, 'B', 0, '3', 0, 'A', 0, '-', 0,                            \
    '7', 0, 'D', 0, '4', 0, 'C', 0, '4', 0, 'B', 0, '4', 0, 'E', 0, '6', 0, 'C', 0, '0', 0, (_last), 0, \
    '}', 0, 0, 0, 0, 0

__ALIGN_BEGIN const uint8_t USBD_WinUSBDescriptorSetDescriptor[USBD_WINUSB_DESC_SET_LEN] __ALIGN_END = {
    /* ---- set header ---- */
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE),
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE),
    0x00, 0x00, 0x03, 0x06,                   /* dwWindowsVersion >= Win 8.1 */
    WBVAL(USBD_WINUSB_DESC_SET_LEN),
    /* ---- subset A: IF0 命令通道 ---- */
    EXLINK_WINUSB_FUNC_SUBSET(EXLINK_IFNUM_CMD, '1'),
    /* ---- subset B: IF1 CMSIS-DAP v2 ---- */
    EXLINK_WINUSB_FUNC_SUBSET(EXLINK_IFNUM_DAP, '2'),
};

/* ---- BOS descriptor (仅 WinUSB platform capability) ---- */
#define USBD_WINUSB_DESC_LEN  28
#define USBD_BOS_WTOTALLENGTH (0x05 + USBD_WINUSB_DESC_LEN)

__ALIGN_BEGIN const uint8_t USBD_BinaryObjectStoreDescriptor[USBD_BOS_WTOTALLENGTH] __ALIGN_END = {
    0x05,                         /* bLength */
    0x0f,                         /* bDescriptorType = BOS */
    WBVAL(USBD_BOS_WTOTALLENGTH), /* wTotalLength */
    0x01,                         /* bNumDeviceCaps */

    USBD_WINUSB_DESC_LEN,           /* bLength */
    0x10,                           /* DEVICE CAPABILITY */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0xDF, 0x60, 0xDD, 0xD8,         /* MS OS 2.0 PlatformCapabilityUUID */
    0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D,
    0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06,         /* dwWindowsVersion >= Win 8.1 */
    WBVAL(USBD_WINUSB_DESC_SET_LEN),
    EXLINK_WINUSB_VENDOR_CODE,      /* bVendorCode */
    0,                              /* bAltEnumCode */
};

static struct usb_msosv2_descriptor msosv2_desc = {
    .vendor_code = EXLINK_WINUSB_VENDOR_CODE,
    .compat_id = USBD_WinUSBDescriptorSetDescriptor,
    .compat_id_len = USBD_WINUSB_DESC_SET_LEN,
};

static struct usb_bos_descriptor bos_desc = {
    .string = USBD_BinaryObjectStoreDescriptor,
    .string_len = USBD_BOS_WTOTALLENGTH,
};

/* ===========================================================================
 *  字符串描述符
 * ========================================================================= */
static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* 0: LANGID (en-US) */
    "LabForge",                   /* 1: Manufacturer */
    "ExLink",                     /* 2: Product (设备总名) */
    "EXLINK-F1-0001",             /* 3: Serial */
    "ExLink Command",             /* 4: iInterface IF0 (vendor 命令通道) */
    "ExLink CMSIS-DAP v2",        /* 5: iInterface IF1 (CMSIS-DAP v2 bulk) */
    "ExLink CMSIS-DAP v1",        /* 6: iInterface IF2 (CMSIS-DAP v1 HID) */
};
#define EXLINK_STR_IDX_MAX  6u    /* 字符串描述符最大有效索引 */

/* ===========================================================================
 *  advance-desc 回调
 * ========================================================================= */
static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}
static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    return (speed == USB_SPEED_HIGH) ? config_descriptor_hs : config_descriptor_fs;
}
static const uint8_t *other_speed_descriptor_callback(uint8_t speed)
{
    /* other-speed = 当前速度的"另一速度" */
    return (speed == USB_SPEED_HIGH) ? other_speed_descriptor_fs : other_speed_descriptor_hs;
}
static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}
static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index > EXLINK_STR_IDX_MAX) {
        return NULL;
    }
    return string_descriptors[index];
}

static const struct usb_descriptor exlink_composite_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
    .msosv2_descriptor = &msosv2_desc,
    .bos_descriptor = &bos_desc,
};

/* ===========================================================================
 *  USB 事件分发: 把 RESET / CONFIGURED 广播给各功能模块
 * ========================================================================= */
static void exlink_usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
    case USBD_EVENT_RESET:
    case USBD_EVENT_DISCONNECTED:
        usb_if_ctrl_on_reset();
        usb_if_dap_on_reset();
        usb_if_dapv1_on_reset();
        usb_if_uart_on_reset();
        break;
    case USBD_EVENT_CONFIGURED:
        usb_if_ctrl_on_configured(busid);
        usb_if_dap_on_configured(busid);
        usb_if_dapv1_on_configured(busid);
        usb_if_uart_on_configured(busid);
        break;
    default:
        break;
    }
}

/* ===========================================================================
 *  装配: 注册描述符 -> 按接口号顺序挂各模块接口/端点 -> 初始化
 *  注册顺序**必须**与 config 描述符里的接口顺序一致 (接口号按 add 顺序自增)。
 * ========================================================================= */
void exlink_usb_init(uint8_t busid, uint32_t reg_base)
{
    usbd_desc_register(busid, &exlink_composite_descriptor);

    usb_if_ctrl_add(busid);    /* IF0      (+EP1 in/out)              */
    usb_if_dap_add(busid);     /* IF1      (+EP2 in/out, DAP-v2 bulk) */
    usb_if_dapv1_add(busid);   /* IF2      (+EP3 in/out, DAP-v1 HID)  */
    usb_if_uart_add(busid);    /* IF3,IF4(+EP4/5) 然后 IF5,IF6(+EP6/7) */

    usbd_initialize(busid, reg_base, exlink_usbd_event_handler);
}
