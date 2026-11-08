#include "hal_data.h"
#include <stdio.h>
#include <string.h>

#if (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
bsp_ipc_semaphore_handle_t g_core_start_semaphore =
{
    .semaphore_num = 0
};
#endif

/* === 参数 === */
#define GPT_PERIOD_COUNTS   (4500U)   /* 20kHz @ 90MHz PCLKD */
#define DUTY_LIMIT          (100)     /* 占空比限幅 ±100% */
#define DEAD_ZONE           (55)      /* 启动死区前馈：AT8236 实测约 55% 才转，目标非0时输出保底此值 */
#define LOW_SPEED_TH        (150)     /* 平滑启动判定：速度低于此视为刚起步，输出固定前馈（不叠PID增量） */
#define SPEED_LIMIT         (3000)    /* 目标速度限幅 ±3000（每500ms计数） */

/* === PID 参数（左右轮共用同一套，实测整定值） === */
#define PID_UPDATE_MS       (20U)     /* PID 更新周期：每 20ms 算一次 */
#define SPEED_SCALE         (500U / PID_UPDATE_MS)  /* 20ms 差分换算成"每500ms" = 25 */
#define PID_I_LIMIT         (10000)   /* 积分项限幅（抗积分饱和） */
#define FILTER_ALPHA        (0.4f)    /* 速度低通滤波系数（抗编码器噪声，更平滑） */

#define PID_KP_DEFAULT      (9)       /* Kp = 0.09 */
#define PID_KI_DEFAULT      (1)       /* Ki = 0.01 */
#define PID_KD_DEFAULT      (0)       /* Kd = 0 */

/* === CAN 协议（v0.2, 500kbps, 经典帧 DLC=8, 小端） === */
#define CAN_ID_CMD        (0x180U)   /* 上→下 速度/模式/使能 @50Hz */
#define CAN_ID_PARAM_REQ  (0x181U)   /* 上→下 参数读写请求 */
#define CAN_ID_ODO        (0x280U)   /* 下→上 累计计数+状态 @100Hz */
#define CAN_ID_PARAM_RSP  (0x282U)   /* 下→上 参数应答 */
#define CAN_ID_STATUS     (0x300U)   /* 下→上 电压/温度/故障 @10Hz */

/* === CAN 状态机与限幅（协议 v0.2 §4-§5） === */
#define CAN_TIMEOUT_MS    (200U)     /* 200ms 无有效 0x180 → 停车回 IDLE（协议 §8） */
#define CAN_MAX_V_MM_S    (600)      /* 线速度上限 mm/s（协议参数 idx4 默认） */
#define CAN_MAX_W_MRAD_S  (2000)     /* 角速度上限 mrad/s（协议参数 idx5 默认） */
#define CAN_STATE_IDLE    (0U)       /* 0 IDLE */
#define CAN_STATE_TELEOP  (1U)       /* 1 TELEOP */
#define CAN_STATE_AUTO    (2U)       /* 2 AUTO */
#define CAN_STATE_FAULT   (3U)       /* 3 FAULT */
/* 硬件 TELEOP/AUTO 模式开关校验：默认 0=不校验（小车无此物理开关，接受请求）。
   若以后加开关，置 1 并实现 can_mode_sw_ok()（协议 §5.4：不匹配 → FAULT err=9） */
#define CAN_MODE_SW_EN    (0)
#define CAN_PROTO_VER     (2U)                          /* 协议版本 v0.2 */
#define CAN_FW_VERSION    (0x01000000U)                 /* fw v1.0.0.0 */
#define CAN_COUNTS_PER_REV (1174)   /* 实际：13PPR×4×22.569≈1174（协议参数 idx1，文档默认1320为旧假设） */
#define CAN_TRACK_MM      (121)     /* 轮距 121.4mm → 0x181 返回整数 121（协议参数 idx3） */
#define CAN_WHEEL_MM      (65)      /* 轮径 65mm（协议参数 idx2） */

/* === 控制源模式（蓝牙/串口 vs 上位机 CAN 互斥） ===
   蓝牙模式下忽略上位机 0x180 运动命令，防止其停车帧覆盖蓝牙目标；
   退出该模式（M:1）后才接收上位机控制。P405 作模式指示 LED。 */
#define CTL_MODE_UART     (0)       /* 蓝牙/串口优先（忽略 CAN 0x180） */
#define CTL_MODE_CAN      (1)       /* 上位机优先（正常处理 0x180） */
#define CTL_MODE_DEFAULT  (CTL_MODE_UART)   /* 上电默认蓝牙/串口优先 */
#define LED_PORT          (R_PORT4)         /* P405：模式指示 LED（端口4） */
#define LED_PIN_BIT       (5U)              /* P405 = 端口4 bit5 */
#define LED_ON            (1)               /* 蓝牙/串口模式=亮 */
#define LED_OFF           (0)               /* 上位机 CAN 模式=灭 */

/* 速度/里程换算（实物标定：13PPR×4×22.569≈1174/轮转，轮径65mm，轮距121.4mm） */
#define MM_TO_CNT_Q       (2873)     /* 1mm/s ≈ 2.873 速度单位（每500ms计数）×1000 */
#define TRACK_W_MM_Q      (1214)     /* 轮距 121.4mm ×10 */

/* === 电机/控制变量 === */
static volatile int32_t  g_left_target  = 0;   /* 左轮目标速度（独立） */
static volatile int32_t  g_right_target = 0;   /* 右轮目标速度（独立） */
static int32_t g_left_ramp  = 0;   /* 左轮斜坡后目标 */
static int32_t g_right_ramp = 0;   /* 右轮斜坡后目标 */
static volatile int32_t  g_left_duty    = 0;   /* PID 算出的左轮占空比 */
static volatile int32_t  g_right_duty   = 0;   /* PID 算出的右轮占空比 */
static volatile int32_t  g_speed_l      = 0;   /* 左轮实际速度（滤波后） */
static volatile int32_t  g_speed_r      = 0;   /* 右轮实际速度 */
static volatile int32_t  g_kp_int       = PID_KP_DEFAULT;  /* Kp×100 */
static volatile int32_t  g_ki_int       = PID_KI_DEFAULT;  /* Ki×100 */
static volatile int32_t  g_kd_int       = PID_KD_DEFAULT;  /* Kd×100 */
static volatile uint32_t g_last_cmd_ms  = 0;
static volatile uint8_t  g_ctl_mode = CTL_MODE_DEFAULT;  /* 控制源：蓝牙/串口 或 上位机 CAN */

/* === CAN 变量 === */
static volatile uint32_t g_can_age_ms = 0;   /* 距最后 0x180 的 ms，饱和 65535 */
static volatile int16_t  g_can_v_ref  = 0;   /* mm/s */
static volatile int16_t  g_can_w_ref  = 0;   /* mrad/s */
static volatile uint8_t  g_can_mode   = 0;
static volatile uint8_t  g_can_flags  = 0;   /* bit0 enable */
static volatile uint8_t  g_can_status = 0;   /* 0x280.status */
static volatile uint8_t  g_can_state   = CAN_STATE_IDLE;  /* 模式状态机（IDLE/TELEOP/AUTO/FAULT） */
static volatile uint8_t  g_can_prev_en = 0;   /* enable 上次电平（0→1 边沿检测） */
static volatile uint8_t  g_can_prev_flags = 0; /* flags 上次值（clear_fault 边沿） */
static volatile uint8_t  g_can_fault   = 0;   /* 0x300.err 首个故障码（0=无） */
static volatile uint8_t  g_can_bus_err = 0;   /* 0x300.can_err（总线错误位） */
static uint32_t g_can_seq      = 0;          /* 发送帧 seq */
static uint32_t g_boot_id      = 1;          /* 复位 +1 */
static int32_t  g_enc_cum_l = 0, g_enc_cum_r = 0;  /* 上电累计编码器计数 */
static volatile uint32_t g_tick_ms = 0;   /* 系统 tick（ms） */

/* PID 状态（积分、上次误差）——左右轮各自独立，保证两轮分别闭环 */
static int32_t g_il = 0, g_prev_err_l = 0;
static int32_t g_ir = 0, g_prev_err_r = 0;

/* 收到目标/增益命令时清零积分与上次误差，防止 wind-up 在切换时踢车 */
static void pid_reset_state(void)
{
    g_il = 0; g_prev_err_l = 0;
    g_ir = 0; g_prev_err_r = 0;
}

/* === CAN 发送/换算 === */
static int32_t mm_s_to_units(int32_t mm_s)
{
    return (int32_t)(((int64_t) mm_s * MM_TO_CNT_Q) / 1000);
}

static void can_send(uint32_t mailbox, uint32_t id, const uint8_t d[8])
{
    can_frame_t fr;
    memset(&fr, 0, sizeof(fr));
    fr.id                = id;
    fr.id_mode           = CAN_ID_MODE_STANDARD;
    fr.type              = CAN_FRAME_TYPE_DATA;
    fr.data_length_code  = 8U;
    memcpy(fr.data, d, 8);
    g_can0.p_api->write(g_can0.p_ctrl, mailbox, &fr);
}

/* 0x280：累计计数 + 采样时刻 + 状态 @100Hz，走 TX FIFO */
static void can_send_280(void)
{
    uint8_t d[8];
    uint32_t t = g_tick_ms & 0xFFFFU;
    d[0] = (uint8_t)(t & 0xFF);
    d[1] = (uint8_t)((t >> 8) & 0xFF);
    d[2] = (uint8_t)((uint16_t) g_enc_cum_l & 0xFF);
    d[3] = (uint8_t)(((uint16_t) g_enc_cum_l >> 8) & 0xFF);
    d[4] = (uint8_t)((uint16_t) g_enc_cum_r & 0xFF);
    d[5] = (uint8_t)(((uint16_t) g_enc_cum_r >> 8) & 0xFF);
    d[6] = (uint8_t) g_can_status;
    d[7] = (uint8_t)(g_can_seq++ & 0xFF);
    can_send(CAN_MAILBOX_ID_TX_FIFO, CAN_ID_ODO, d);
}

/* 0x300：电压/温度/故障/复位/总线 @10Hz，走 TX FIFO（vbat/temp 固定占位值） */
static void can_send_300(void)
{
    uint8_t d[8];
    uint32_t age = (g_can_age_ms > 65535U) ? 65535U : g_can_age_ms;
    uint32_t vbat = 12000U;   /* 12.0V 固定占位值（不接 ADC，按实际写死） */
    d[0] = (uint8_t)(vbat & 0xFF);
    d[1] = (uint8_t)((vbat >> 8) & 0xFF);
    d[2] = 25;                /* 25°C 固定占位值（不接温度测量） */
    d[3] = (uint8_t) g_can_fault;   /* err：真实故障码（bus-off=7 等） */
    d[4] = (uint8_t)(g_boot_id & 0xFF);
    d[5] = (uint8_t) g_can_bus_err; /* can_err：总线错误位（真实） */
    d[6] = (uint8_t)(age & 0xFF);
    d[7] = (uint8_t)((age >> 8) & 0xFF);
    can_send(CAN_MAILBOX_ID_TX_FIFO, CAN_ID_STATUS, d);
}

/* 速度/角速度限幅 */
static int16_t can_clamp_i16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { return (int16_t) lo; }
    if (v > hi) { return (int16_t) hi; }
    return (int16_t) v;
}

/* 组装 0x280.status（协议 §5.1） */
static void can_update_status(void)
{
    uint8_t s = (uint8_t)(g_can_state & 3U);
    if (g_can_age_ms > CAN_TIMEOUT_MS) { s |= (1U << 2); }  /* cmd_timeout */
    if (g_can_flags & 0x04U)           { s |= (1U << 7); }  /* soft_estop */
    g_can_status = s;
}

/* 0x282：参数应答（协议 §4.4）。status: 0 OK / 1 未知索引 / 2 超范围 / 3 AUTO锁定 */
static void can_send_282(uint8_t idx, uint8_t op, int32_t value, uint8_t status, uint8_t seq)
{
    uint8_t d[8];
    d[0] = idx;
    d[1] = op;
    d[2] = (uint8_t)((uint32_t) value & 0xFF);
    d[3] = (uint8_t)(((uint32_t) value >> 8) & 0xFF);
    d[4] = (uint8_t)(((uint32_t) value >> 16) & 0xFF);
    d[5] = (uint8_t)(((uint32_t) value >> 24) & 0xFF);
    d[6] = status;
    d[7] = seq;
    can_send(CAN_MAILBOX_ID_TX_FIFO, CAN_ID_PARAM_RSP, d);
}

/* 参数读写（协议 §6）。写只允许 IDLE；返回读到/写后的值，*status 置应答码。
   kp/ki 为 kp_milli/ki_milli（×1000），读写时与 g_kp_int/g_ki_int(×100) 换算。 */
static int32_t can_param_access(uint8_t idx, uint8_t op, int32_t value, uint8_t *status)
{
    int32_t r = 0;
    *status = 0U;
    switch (idx)
    {
        case 0:  r = CAN_PROTO_VER;      break;                  /* proto_ver 只读 */
        case 1:  r = CAN_COUNTS_PER_REV; break;                  /* counts_per_rev 实际 1174 */
        case 2:  r = CAN_WHEEL_MM;       break;                  /* wheel_diameter_mm */
        case 3:  r = CAN_TRACK_MM;       break;                  /* track_width_mm 实际 121 */
        case 4:  r = CAN_MAX_V_MM_S;     break;
        case 5:  r = CAN_MAX_W_MRAD_S;   break;
        case 6:  r = 500;                break;                  /* accel_mm_s2 */
        case 7:  r = 4000;               break;                  /* alpha_mrad_s2 */
        case 8:  r = g_kp_int * 10;      break;                  /* kp_milli */
        case 9:  r = g_ki_int * 10;      break;                  /* ki_milli */
        case 10: r = 3000;               break;                  /* current_limit_ma */
        case 11: r = CAN_TIMEOUT_MS;     break;                  /* cmd_timeout_ms */
        case 12: r = 500;                break;                  /* can_bitrate 只读 */
        case 13: r = CAN_FW_VERSION;     break;                  /* fw_version 只读 */
        case 14: r = 0;                  break;                  /* encoder_sign */
        default: *status = 1U; return 0;                         /* 未知索引 */
    }
    if (op == 1)   /* 写：只允许 IDLE */
    {
        if (g_can_state != CAN_STATE_IDLE) { *status = 3U; return r; }
        if (idx == 8)  { g_kp_int = value / 10; }
        else if (idx == 9) { g_ki_int = value / 10; }
        else           { *status = 2U; return r; }               /* 只读/超范围 */
        r = (idx == 8) ? (g_kp_int * 10) : (g_ki_int * 10);
        pid_reset_state();
    }
    return r;
}

#if (CAN_MODE_SW_EN)
/* 硬件 TELEOP/AUTO 模式开关校验（协议 §5.4）：请求与开关不一致 → 返回 false。
   当前小车无此开关，占位恒返回 true；接 GPIO 后在此实现。 */
static bool can_mode_sw_ok(uint8_t mode)
{
    (void) mode;
    return true;
}
#endif

/* === 串口接收缓冲（空闲超时解析，无需换行符） === */
static char          g_rx_buf[64];
static volatile uint8_t  g_rx_idx = 0;
static volatile uint8_t  g_rx_line_ready = 0;   /* 收到 \r\n → 整条命令就绪 */
static volatile uint32_t g_rx_last_char_ms = 0;
#define RX_PARSE_IDLE_MS (20U)
#define RX_TIMEOUT_MS    (200U)   /* 接收超时停车：200ms 无指令自动停 */
#define RAMP_STEP        (150)    /* 目标速度斜坡步长：每 PID 周期(20ms)最多变化150 */

/* === PID 控制器：duty = Kp*err + Ki*∫err + Kd*Δerr，带抗饱和+输出限幅 === */
static int32_t pid_calc(int32_t target, int32_t actual,
                        int32_t kp_int, int32_t ki_int, int32_t kd_int,
                        int32_t *integral, int32_t *prev_err)
{
    int32_t err = target - actual;
    int32_t derr = err - *prev_err;      /* 误差变化率（微分） */
    *prev_err = err;

    float kp = (float) kp_int / 100.0f;
    float ki = (float) ki_int / 100.0f;
    float kd = (float) kd_int / 100.0f;
    float out = kp * (float) err + ki * (float) (*integral) + kd * (float) derr;

    /* 抗积分饱和：输出已饱和且误差同向 → 不累加积分 */
    if (!(out > (float) DUTY_LIMIT && err > 0) && !(out < (float) -DUTY_LIMIT && err < 0))
    {
        *integral += err;
    }
    if (*integral > PID_I_LIMIT)  { *integral = PID_I_LIMIT; }
    if (*integral < -PID_I_LIMIT) { *integral = -PID_I_LIMIT; }

    /* 全幅限幅 -100~100：允许 PID 增量反向以便超调减速；
       最终单象限由调用处叠加死区前馈后再钳制（电机不会真正反转） */
    if (out > (float) DUTY_LIMIT)  { out = (float) DUTY_LIMIT; }
    if (out < (float) -DUTY_LIMIT) { out = (float) -DUTY_LIMIT; }
    return (int32_t) out;
}

/* === 解析命令 === */
/* L:%d R:%d / L:%d / R:%d → 设目标速度（左右轮独立） */
/* Kp:%d / Ki:%d / Kd:%d  → 调共用 PID 参数（均×100） */
/* T:%d                   → 原地旋转。正=逆时针(左反右正)，负=顺时针，0=停 */

/* === P405 模式指示 LED：蓝牙/串口=亮，上位机 CAN=灭（直接驱动端口寄存器） === */
static void led_set_level(int on)
{
    if (on) { LED_PORT->PODR_b.PODR5 = 1; }
    else    { LED_PORT->PODR_b.PODR5 = 0; }
}

static void led_init(void)
{
    LED_PORT->PDR_b.PDR5 = 1;   /* P405 设为输出 */
    led_set_level((g_ctl_mode == CTL_MODE_CAN) ? LED_OFF : LED_ON);
}

static void led_update_mode(void)
{
    led_set_level((g_ctl_mode == CTL_MODE_CAN) ? LED_OFF : LED_ON);
}

/* 回显同时发往有线(SCI9)和无线(SCI0)，方便无线链路验证 */
static void ack_both(const char *s, uint32_t n)
{
    g_uart0.p_api->write(g_uart0.p_ctrl, (uint8_t *)s, n);
    g_uart1.p_api->write(g_uart1.p_ctrl, (uint8_t *)s, n);
}

static void parse_rx_line(void)
{
    g_rx_buf[g_rx_idx] = '\0';
    int32_t l = 0, r = 0, v = 0;
    char ack[40];

    if (2 == sscanf(g_rx_buf, "L:%d R:%d", &l, &r))
    {
        if (l >= -SPEED_LIMIT && l <= SPEED_LIMIT) { g_left_target = l; }
        if (r >= -SPEED_LIMIT && r <= SPEED_LIMIT) { g_right_target = r; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK L:%d R:%d\r\n", g_left_target, g_right_target);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "L:%d", &l))
    {
        if (l >= -SPEED_LIMIT && l <= SPEED_LIMIT) { g_left_target = l; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK L:%d\r\n", g_left_target);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "R:%d", &r))
    {
        if (r >= -SPEED_LIMIT && r <= SPEED_LIMIT) { g_right_target = r; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK R:%d\r\n", g_right_target);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "Kp:%d", &v))
    {
        if (v >= 1 && v <= 500) { g_kp_int = v; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK Kp:%d (=%.2f)\r\n", g_kp_int, (float)g_kp_int / 100.0f);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "Ki:%d", &v))
    {
        if (v >= 0 && v <= 200) { g_ki_int = v; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK Ki:%d (=%.2f)\r\n", g_ki_int, (float)g_ki_int / 100.0f);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "Kd:%d", &v))
    {
        if (v >= 0 && v <= 200) { g_kd_int = v; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK Kd:%d (=%.2f)\r\n", g_kd_int, (float)g_kd_int / 100.0f);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "T:%d", &v))
    {
        /* T:速度 → 原地旋转。正=逆时针(左反右正)，负=顺时针，0=停 */
        if (v >= -SPEED_LIMIT && v <= SPEED_LIMIT && v != 0)
        {
            g_left_target  = -v;
            g_right_target =  v;
        }
        else
        {
            g_left_target  = 0;
            g_right_target = 0;
        }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK T:%d\r\n", v);
        ack_both(ack, n);
    }
    else if (1 == sscanf(g_rx_buf, "M:%d", &v))
    {
        /* 控制源切换：M:0=蓝牙/串口优先（忽略 CAN 0x180），M:1=上位机优先。切换即停车 */
        if (v == 0)      { g_ctl_mode = CTL_MODE_UART; }
        else if (v == 1) { g_ctl_mode = CTL_MODE_CAN; }
        g_left_target = 0; g_right_target = 0;
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        led_update_mode();
        uint32_t n = (uint32_t) sprintf(ack, "OK M:%d (%s)\r\n", g_ctl_mode,
                                        (g_ctl_mode == CTL_MODE_CAN) ? "CAN" : "UART");
        ack_both(ack, n);
    }
    g_rx_idx = 0;
}

/* === 串口回调：逐字节累积到缓冲 === */
/* user_uart_callback  = SCI9 (P109/P110, 有线调试) */
/* wireless_uart_callback = SCI0 (P100/P101, MX-02P 无线) */
/* 两个串口的 RX 都累积到同一个 g_rx_buf，共用同一套命令解析 */
void user_uart_callback(uart_callback_args_t *p_args)
{
    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        char c = (char) p_args->data;
        if (c == '\r' || c == '\n')
        {
            g_rx_line_ready = 1;   /* 回车 → 整条命令就绪（蓝牙分段也不拆开） */
        }
        else if (g_rx_idx < sizeof(g_rx_buf) - 1)
        {
            g_rx_buf[g_rx_idx++] = c;
            g_rx_last_char_ms = g_tick_ms;
            g_rx_line_ready = 0;   /* 有新内容，等待真正结束 */
        }
    }
}

void wireless_uart_callback(uart_callback_args_t *p_args)
{
    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        char c = (char) p_args->data;
        if (c == '\r' || c == '\n')
        {
            g_rx_line_ready = 1;   /* 回车 → 整条命令就绪 */
        }
        else if (g_rx_idx < sizeof(g_rx_buf) - 1)
        {
            g_rx_buf[g_rx_idx++] = c;
            g_rx_last_char_ms = g_tick_ms;
            g_rx_line_ready = 0;
        }
    }
}

/* === CAN 接收回调（r_can 纯中断接收，帧在 p_args->frame） === */
void can_callback(can_callback_args_t *p_args)
{
    /* 总线/错误事件（不带 RX_COMPLETE）：先处理，否则会被下面 return 拦截 */
    if (p_args->event & CAN_EVENT_ERR_WARNING)  { g_can_bus_err |= 0x01U; }
    if (p_args->event & CAN_EVENT_ERR_PASSIVE)  { g_can_bus_err |= 0x02U; }
    if (p_args->event & CAN_EVENT_ERR_BUS_OFF)
    {
        g_can_bus_err |= 0x04U;
        g_can_state  = CAN_STATE_FAULT;
        if (g_can_fault == 0) { g_can_fault = 7; }   /* err=7 CAN bus-off */
        g_left_target = 0; g_right_target = 0;
    }
    if (p_args->event & CAN_EVENT_BUS_RECOVERY) { g_can_bus_err &= (uint8_t) ~0x04U; }
    if (p_args->event & CAN_EVENT_MAILBOX_MESSAGE_LOST) { g_can_bus_err |= 0x08U; }  /* rx_overflow */

    if (0 == (p_args->event & CAN_EVENT_RX_COMPLETE)) { return; }
    can_frame_t *f = &p_args->frame;
    uint16_t id = (uint16_t)(f->id & 0x7FFU);

    /* CAN 收帧转发串口（有线 COM4 观察） */
    {
        char b[48];
        uint32_t n = (uint32_t) sprintf(b, "CAN RX %03X: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                          (unsigned) id,
                          f->data[0], f->data[1], f->data[2], f->data[3],
                          f->data[4], f->data[5], f->data[6], f->data[7]);
        g_uart0.p_api->write(g_uart0.p_ctrl, (uint8_t *)b, n);
    }

    if (CAN_ID_CMD == id)      /* 0x180：v_ref(mm/s)/w_ref(mrad/s)/mode/flags/seq 小端 */
    {
        if (g_ctl_mode != CTL_MODE_CAN)
        {
            /* 蓝牙/串口优先：忽略上位机 0x180 运动命令，防止其停车帧覆盖蓝牙目标。
               g_can_age_ms 不更新 → 恒超时 → 停车只看串口超时 */
            return;
        }
        int16_t v = (int16_t)(f->data[0] | ((uint16_t) f->data[1] << 8));
        int16_t w = (int16_t)(f->data[2] | ((uint16_t) f->data[3] << 8));
        uint8_t mode  = f->data[4];
        uint8_t flags = f->data[5];
        uint8_t en    = flags & 0x01U;

        g_can_v_ref = can_clamp_i16(v, -CAN_MAX_V_MM_S, CAN_MAX_V_MM_S);
        g_can_w_ref = can_clamp_i16(w, -CAN_MAX_W_MRAD_S, CAN_MAX_W_MRAD_S);
        g_can_mode  = mode;
        g_can_flags = flags;
        g_can_age_ms = 0;             /* 距最后 0x180，独立于串口 */

        /* enable 0→1 上升沿：仅从 IDLE 进入运动模式（协议 §7.6：不能把长高电平当新 0→1） */
        if (en && !g_can_prev_en)
        {
            if (g_can_state == CAN_STATE_IDLE && (mode == CAN_STATE_TELEOP || mode == CAN_STATE_AUTO))
            {
#if (CAN_MODE_SW_EN)
                if (!can_mode_sw_ok(mode)) { g_can_state = CAN_STATE_FAULT; g_can_fault = 9; }
                else                       { g_can_state = mode; pid_reset_state(); }
#else
                g_can_state = mode;
                pid_reset_state();
#endif
            }
        }
        /* clear_fault 上升沿：FAULT → 静止 → 回 IDLE（协议 §5.4） */
        if ((flags & 0x02U) && !(g_can_prev_flags & 0x02U) && (g_can_state == CAN_STATE_FAULT))
        {
            g_can_state = CAN_STATE_IDLE;
            g_can_fault = 0;
        }

        if (!en)   /* enable=0 → 停车回 IDLE（FAULT 锁存，必须 clear_fault 才能退出，协议 §5.4） */
        {
            if (g_can_state != CAN_STATE_FAULT) { g_can_state = CAN_STATE_IDLE; }
            g_left_target = 0; g_right_target = 0;
            pid_reset_state();
        }
        else if (g_can_state == CAN_STATE_TELEOP || g_can_state == CAN_STATE_AUTO)
        {
            /* 应用差速目标（mm/s 域，协议 §5.1：v_l=v−w·b/2000，v_r=v+w·b/2000）
               持续使能时每帧刷新目标，但不清积分（保证速度环积分稳定建立） */
            int32_t dv = (int32_t)(((int64_t) g_can_w_ref * TRACK_W_MM_Q) / 20000);
            g_left_target  = mm_s_to_units((int32_t) g_can_v_ref - dv);
            g_right_target = mm_s_to_units((int32_t) g_can_v_ref + dv);
        }
        g_can_prev_en    = en;
        g_can_prev_flags = flags;
    }
    else if (CAN_ID_PARAM_REQ == id)   /* 0x181：参数读/写请求 → 0x282 应答 */
    {
        uint8_t idx = f->data[0];
        uint8_t op  = f->data[1];
        int32_t val = (int32_t)((uint32_t)f->data[2] | ((uint32_t)f->data[3] << 8)
                              | ((uint32_t)f->data[4] << 16) | ((uint32_t)f->data[5] << 24));
        uint8_t seq = f->data[6];
        uint8_t st  = 0U;
        int32_t out = can_param_access(idx, op, val, &st);
        can_send_282(idx, op, out, st, seq);
    }
}

/* === 电机输出：duty > 0 正转, < 0 反转, = 0 停 === */
static void motor_apply(const timer_instance_t *timer, int32_t duty, bool left_wheel)
{
    uint32_t counts = GPT_PERIOD_COUNTS * (uint32_t)(duty < 0 ? -duty : duty) / 100U;

    if (left_wheel)
    {
        /* 左轮(GPT1): 正转 = CA=0, CB=PWM（实测方向） */
        if (duty > 0)
        {
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0,      GPT_IO_PIN_GTIOCA);
            timer->p_api->dutyCycleSet(timer->p_ctrl, counts, GPT_IO_PIN_GTIOCB);
        }
        else if (duty < 0)
        {
            timer->p_api->dutyCycleSet(timer->p_ctrl, counts, GPT_IO_PIN_GTIOCA);
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0,     GPT_IO_PIN_GTIOCB);
        }
        else
        {
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0, GPT_IO_PIN_GTIOCA);
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0, GPT_IO_PIN_GTIOCB);
        }
    }
    else
    {
        /* 右轮(GPT2): 正转 = CA=PWM, CB=0（实测方向） */
        if (duty > 0)
        {
            timer->p_api->dutyCycleSet(timer->p_ctrl, counts, GPT_IO_PIN_GTIOCA);
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0,      GPT_IO_PIN_GTIOCB);
        }
        else if (duty < 0)
        {
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0,      GPT_IO_PIN_GTIOCA);
            timer->p_api->dutyCycleSet(timer->p_ctrl, counts, GPT_IO_PIN_GTIOCB);
        }
        else
        {
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0, GPT_IO_PIN_GTIOCA);
            timer->p_api->dutyCycleSet(timer->p_ctrl, 0, GPT_IO_PIN_GTIOCB);
        }
    }
}

/*******************************************************************************************************************/
void hal_entry(void)
{
    g_hal_init();

    fsp_err_t err = g_uart0.p_api->open(g_uart0.p_ctrl, g_uart0.p_cfg);
    if (FSP_SUCCESS != err) { while(1){;} }

    /* 无线串口（SCI0: P100/P101, MX-02P） */
    err = g_uart1.p_api->open(g_uart1.p_ctrl, g_uart1.p_cfg);
    /* 无线打开失败不阻塞主程序（有线/电机仍可用），便于诊断 */

    g_timer1.p_api->open(g_timer1.p_ctrl, g_timer1.p_cfg);
    g_timer2.p_api->open(g_timer2.p_ctrl, g_timer2.p_cfg);
    g_timer1.p_api->start(g_timer1.p_ctrl);
    g_timer2.p_api->start(g_timer2.p_ctrl);

    /* === 编码器初始化（FSP 正规配置） === */
    /* g_timer0 = 右轮编码器(GPT0: P300/P108)，g_timer3 = 左轮编码器(GPT3: P111/P112) */
    g_timer0.p_api->open(g_timer0.p_ctrl, g_timer0.p_cfg);
    g_timer3.p_api->open(g_timer3.p_ctrl, g_timer3.p_cfg);
    g_timer0.p_api->start(g_timer0.p_ctrl);
    g_timer3.p_api->start(g_timer3.p_ctrl);

    /* CAN0 初始化（r_can 经典帧 500kbps, P401=CTX0/P402=CRX0） */
    err = g_can0.p_api->open(g_can0.p_ctrl, g_can0.p_cfg);
    if (FSP_SUCCESS == err)
    {
        g_can0.p_api->modeTransition(g_can0.p_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);
    }

    /* P405 模式指示 LED（蓝牙/串口=亮，上位机 CAN=灭） */
    led_init();

    int32_t prev_l = 0, prev_r = 0;   /* 编码器上次计数值（差分） */
    uint32_t last_pid_ms = 0;
    uint32_t last_can280_ms = 0, last_can300_ms = 0;

    while(1)
    {
        g_tick_ms += 10;

        /* 串口命令解析：收到回车即整条解析（蓝牙分段也不拆开）；空闲超时兜底 */
        if (g_rx_line_ready)
        {
            g_rx_line_ready = 0;
            parse_rx_line();
        }
        else if (g_rx_idx > 0 && (g_tick_ms - g_rx_last_char_ms) >= RX_PARSE_IDLE_MS)
        {
            parse_rx_line();
        }

        /* PID 更新 */
        if (g_tick_ms - last_pid_ms >= PID_UPDATE_MS)
        {
            last_pid_ms = g_tick_ms;

            /* 停车：CAN 与串口两个源都超时才停（任一活跃即保持，兼容并行控制）。
               CAN 用 g_can_age_ms（仅 0x180 清零，独立 200ms）；串口用 g_last_cmd_ms。 */
            if ((g_can_age_ms > CAN_TIMEOUT_MS) && (g_tick_ms - g_last_cmd_ms > RX_TIMEOUT_MS))
            {
                g_left_target  = 0;
                g_right_target = 0;
            }

            int32_t enc_l = (int32_t) R_GPT3->GTCNT;   /* 左轮累计计数 */
            int32_t enc_r = (int32_t) R_GPT0->GTCNT;   /* 右轮累计计数 */
            /* 差分换算成"每500ms"速度；L 侧反转使正转=正速度 */
            int32_t d_l = prev_l - enc_l;   /* L 反转：正转=正 */
            int32_t d_r = enc_r - prev_r;
            prev_l = enc_l;
            prev_r = enc_r;
            int32_t raw_l = d_l * (int32_t) SPEED_SCALE;
            int32_t raw_r = d_r * (int32_t) SPEED_SCALE;
            /* 上电累计编码器计数（供 0x280），符号与速度方向一致 */
            g_enc_cum_l += d_l;
            g_enc_cum_r += d_r;
            /* 一阶低通滤波 */
            g_speed_l = (int32_t)(FILTER_ALPHA * (float) raw_l + (1.0f - FILTER_ALPHA) * (float) g_speed_l);
            g_speed_r = (int32_t)(FILTER_ALPHA * (float) raw_r + (1.0f - FILTER_ALPHA) * (float) g_speed_r);

            /* PID：共用一套参数，左右轮状态独立 */
            /* 目标速度斜坡：加速/减速平滑过渡；停车(target=0)直接到位 */
            int32_t tgt_l = g_left_target,  tgt_r = g_right_target;
            if (g_left_target  == 0) { g_left_ramp  = 0; }
            else if (g_left_ramp < g_left_target)
                g_left_ramp  = (g_left_ramp  + RAMP_STEP > g_left_target)  ? g_left_target  : g_left_ramp  + RAMP_STEP;
            else if (g_left_ramp > g_left_target)
                g_left_ramp  = (g_left_ramp  - RAMP_STEP < g_left_target)  ? g_left_target  : g_left_ramp  - RAMP_STEP;
            if (g_right_target == 0) { g_right_ramp = 0; }
            else if (g_right_ramp < g_right_target)
                g_right_ramp = (g_right_ramp + RAMP_STEP > g_right_target) ? g_right_target : g_right_ramp + RAMP_STEP;
            else if (g_right_ramp > g_right_target)
                g_right_ramp = (g_right_ramp - RAMP_STEP < g_right_target) ? g_right_target : g_right_ramp - RAMP_STEP;
            tgt_l = g_left_ramp;
            tgt_r = g_right_ramp;

            /* PID 增量 + 死区前馈：目标非0立即给 55% 越过启动门槛，PID 只做微调；
               超调时增量反向把 duty 拉回 0 减速，最终钳制单象限不反向 */
            int32_t ff_l = (tgt_l > 0) ? DEAD_ZONE : ((tgt_l < 0) ? -DEAD_ZONE : 0);
            int32_t ff_r = (tgt_r > 0) ? DEAD_ZONE : ((tgt_r < 0) ? -DEAD_ZONE : 0);
            g_left_duty  = pid_calc(tgt_l, g_speed_l, g_kp_int, g_ki_int, g_kd_int, &g_il, &g_prev_err_l) + ff_l;
            g_right_duty = pid_calc(tgt_r, g_speed_r, g_kp_int, g_ki_int, g_kd_int, &g_ir, &g_prev_err_r) + ff_r;
            /* 平滑启动：速度还没真正转起来时，输出固定前馈，暂不叠加 PID 增量，避免起步猛冲 */
            if (tgt_l > 0 && g_speed_l < LOW_SPEED_TH)       { g_left_duty  = DEAD_ZONE; }
            else if (tgt_l < 0 && g_speed_l > -LOW_SPEED_TH) { g_left_duty  = -DEAD_ZONE; }
            if (tgt_r > 0 && g_speed_r < LOW_SPEED_TH)       { g_right_duty  = DEAD_ZONE; }
            else if (tgt_r < 0 && g_speed_r > -LOW_SPEED_TH) { g_right_duty  = -DEAD_ZONE; }
            if (tgt_l > 0)       { if (g_left_duty < 0) g_left_duty = 0; if (g_left_duty > 100) g_left_duty = 100; }
            else if (tgt_l < 0)  { if (g_left_duty > 0) g_left_duty = 0; if (g_left_duty < -100) g_left_duty = -100; }
            else                 { g_left_duty = 0; }
            if (tgt_r > 0)       { if (g_right_duty < 0) g_right_duty = 0; if (g_right_duty > 100) g_right_duty = 100; }
            else if (tgt_r < 0)  { if (g_right_duty > 0) g_right_duty = 0; if (g_right_duty < -100) g_right_duty = -100; }
            else                 { g_right_duty = 0; }
        }

        /* 输出电机（PID 算出的占空比） */
        motor_apply(&g_timer1, g_left_duty,  true);   /* 左轮 GPT1 */
        motor_apply(&g_timer2, g_right_duty, false);  /* 右轮 GPT2 */

        /* 每 200ms 上报：左右轮 实际速度(目标速度) —— 已按要求关闭（联调用 COM4 只看 CAN RX） */
        /* if (g_tick_ms - last_report_ms >= 200U)
        {
            last_report_ms = g_tick_ms;
            uint32_t len = (uint32_t) sprintf(buf, "L:%d(%d) R:%d(%d)\r\n",
                              g_speed_l, g_left_target, g_speed_r, g_right_target);
            g_uart0.p_api->write(g_uart0.p_ctrl, (uint8_t *)buf, len);
        } */

        /* CAN 时基：cmd_age 累加 + status 刷新 + 周期发送 */
        if (g_can_age_ms < 0xFFFFU) { g_can_age_ms += 10U; }
        can_update_status();
        if (g_tick_ms - last_can280_ms >= 10U)  { last_can280_ms  = g_tick_ms; can_send_280(); }  /* 100Hz */
        if (g_tick_ms - last_can300_ms >= 100U) { last_can300_ms = g_tick_ms; can_send_300(); } /* 10Hz */

        R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

#if BSP_TZ_SECURE_BUILD
FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable () { }
FSP_CPP_FOOTER
#endif
