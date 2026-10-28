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
#define SPEED_LIMIT         (3000)    /* 目标速度限幅 ±3000（每500ms计数） */

/* === PID 参数（左右轮共用同一套，实测整定值） === */
#define PID_UPDATE_MS       (20U)     /* PID 更新周期：每 20ms 算一次 */
#define SPEED_SCALE         (500U / PID_UPDATE_MS)  /* 20ms 差分换算成"每500ms" = 25 */
#define PID_I_LIMIT         (10000)   /* 积分项限幅（抗积分饱和） */
#define FILTER_ALPHA        (0.6f)    /* 速度低通滤波系数 */

#define PID_KP_DEFAULT      (9)       /* Kp = 0.09 */
#define PID_KI_DEFAULT      (1)       /* Ki = 0.01 */
#define PID_KD_DEFAULT      (0)       /* Kd = 0 */

/* === CAN 协议（v0.2, 500kbps, 经典帧 DLC=8, 小端） === */
#define CAN_ID_CMD        (0x180U)   /* 上→下 速度/模式/使能 @50Hz */
#define CAN_ID_PARAM_REQ  (0x181U)   /* 上→下 参数读写请求 */
#define CAN_ID_ODO        (0x280U)   /* 下→上 累计计数+状态 @100Hz */
#define CAN_ID_PARAM_RSP  (0x282U)   /* 下→上 参数应答 */
#define CAN_ID_STATUS     (0x300U)   /* 下→上 电压/温度/故障 @10Hz */

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

/* === CAN 变量 === */
static volatile uint32_t g_can_age_ms = 0;   /* 距最后 0x180 的 ms，饱和 65535 */
static volatile int16_t  g_can_v_ref  = 0;   /* mm/s */
static volatile int16_t  g_can_w_ref  = 0;   /* mrad/s */
static volatile uint8_t  g_can_mode   = 0;
static volatile uint8_t  g_can_flags  = 0;   /* bit0 enable */
static volatile uint8_t  g_can_status = 0;   /* 0x280.status */
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

/* 0x300：电压/温度/故障/复位/总线 @10Hz，走 TX FIFO（占位：12.0V/25°C/无故障） */
static void can_send_300(void)
{
    uint8_t d[8];
    uint32_t age = (g_can_age_ms > 65535U) ? 65535U : g_can_age_ms;
    uint32_t vbat = 12000U;   /* 12.0V 占位（未接 ADC） */
    d[0] = (uint8_t)(vbat & 0xFF);
    d[1] = (uint8_t)((vbat >> 8) & 0xFF);
    d[2] = 25;                /* 25°C 占位 */
    d[3] = 0;                 /* err = 0 无故障 */
    d[4] = (uint8_t)(g_boot_id & 0xFF);
    d[5] = 0;                 /* can_err */
    d[6] = (uint8_t)(age & 0xFF);
    d[7] = (uint8_t)((age >> 8) & 0xFF);
    can_send(CAN_MAILBOX_ID_TX_FIFO, CAN_ID_STATUS, d);
}

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

    /* 单象限限幅：目标>0 只允许 0~100，目标<0 只允许 -100~0，目标=0 停 */
    /* 运动过程中不允许反向（避免电机来回反转抖动、卡顿） */
    if (target > 0)
    {
        if (out < 0.0f)                { out = 0.0f; }
        if (out > (float) DUTY_LIMIT)  { out = (float) DUTY_LIMIT; }
    }
    else if (target < 0)
    {
        if (out > 0.0f)                { out = 0.0f; }
        if (out < (float) -DUTY_LIMIT) { out = (float) -DUTY_LIMIT; }
    }
    else
    {
        out = 0.0f;
    }
    return (int32_t) out;
}

/* === 解析命令 === */
/* L:%d R:%d / L:%d / R:%d → 设目标速度（左右轮独立） */
/* Kp:%d / Ki:%d / Kd:%d  → 调共用 PID 参数（均×100） */
/* T:%d                   → 原地旋转。正=逆时针(左反右正)，负=顺时针，0=停 */

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
        int16_t v = (int16_t)(f->data[0] | ((uint16_t) f->data[1] << 8));
        int16_t w = (int16_t)(f->data[2] | ((uint16_t) f->data[3] << 8));
        g_can_v_ref = v; g_can_w_ref = w;
        g_can_mode  = f->data[4];
        g_can_flags = f->data[5];
        g_can_age_ms = 0;
        g_last_cmd_ms = g_tick_ms;   /* 与串口共用 200ms 超时 */

        if (g_can_flags & 0x01U)     /* enable=1 → 应用差速目标 */
        {
            int32_t dv = (int32_t)(((int64_t) w * TRACK_W_MM_Q) / 20000);
            g_left_target  = mm_s_to_units((int32_t) v - dv);
            g_right_target = mm_s_to_units((int32_t) v + dv);
        }
        else                          /* enable=0 → 停车 */
        {
            g_left_target  = 0;
            g_right_target = 0;
        }
    }
    else if (CAN_ID_PARAM_REQ == id)   /* 0x181 参数通道：暂不实现 */
    {
        /* TODO 0x181/0x282 */
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

    int32_t prev_l = 0, prev_r = 0;   /* 编码器上次计数值（差分） */
    uint32_t last_pid_ms = 0;
    uint32_t last_report_ms = 0;
    uint32_t last_can280_ms = 0, last_can300_ms = 0;
    char buf[64];

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

            /* 接收超时停车：500ms 无任何串口指令 → 自动停车 */
            /* 遥控场景下，松开方向键后 MCU 不再收指令，会自动停（不依赖停车指令传不传得到） */
            if (g_tick_ms - g_last_cmd_ms > RX_TIMEOUT_MS)
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

            g_left_duty  = pid_calc(tgt_l, g_speed_l, g_kp_int, g_ki_int, g_kd_int, &g_il, &g_prev_err_l);
            g_right_duty = pid_calc(tgt_r, g_speed_r, g_kp_int, g_ki_int, g_kd_int, &g_ir, &g_prev_err_r);
        }

        /* 输出电机（PID 算出的占空比） */
        motor_apply(&g_timer1, g_left_duty,  true);   /* 左轮 GPT1 */
        motor_apply(&g_timer2, g_right_duty, false);  /* 右轮 GPT2 */

        /* 每 200ms 上报：左右轮 实际速度(目标速度) */
        if (g_tick_ms - last_report_ms >= 200U)
        {
            last_report_ms = g_tick_ms;
            uint32_t len = (uint32_t) sprintf(buf, "L:%d(%d) R:%d(%d)\r\n",
                              g_speed_l, g_left_target, g_speed_r, g_right_target);
            g_uart0.p_api->write(g_uart0.p_ctrl, (uint8_t *)buf, len);
        }

        /* CAN 时基：cmd_age 累加 + 周期发送 */
        if (g_can_age_ms < 0xFFFFU) { g_can_age_ms += 10U; }
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
