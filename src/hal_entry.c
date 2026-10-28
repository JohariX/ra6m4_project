#include "hal_data.h"
#include <stdio.h>


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

/* PID 状态（积分、上次误差）——左右轮各自独立，保证两轮分别闭环 */
static int32_t g_il = 0, g_prev_err_l = 0;
static int32_t g_ir = 0, g_prev_err_r = 0;

/* 收到目标/增益命令时清零积分与上次误差，防止 wind-up 在切换时踢车 */
static void pid_reset_state(void)
{
    g_il = 0; g_prev_err_l = 0;
    g_ir = 0; g_prev_err_r = 0;
}

/* === 串口接收缓冲：有线(SCI9)与无线(SCI0)各自独立，避免两通道并发覆盖 === */
typedef struct {
    char  buf[64];
    volatile uint8_t  idx;          /* 已收字节数 */
    volatile uint8_t  line_ready;   /* 收到 \r\n → 整条命令就绪 */
    volatile uint32_t last_char_ms; /* 最近一次收字符的 tick */
} rx_chan_t;

static rx_chan_t g_rx_uart0 = {0};  /* 有线 SCI9 (P109/P110) */
static rx_chan_t g_rx_uart1 = {0};  /* 无线 SCI0 (P100/P101, MX-02P) */
#define RX_PARSE_IDLE_MS (20U)
#define RX_TIMEOUT_MS    (200U)   /* 接收超时停车：200ms 无指令自动停 */
#define RAMP_STEP        (150)    /* 目标速度斜坡步长：每 PID 周期(20ms)最多变化150 */

/* === CAN 协议 v0.2 (500kbps 经典帧 DLC=8, 对接 RA8P1) === */
#define CAN_TX_MB        (0U)          /* 发送用 mailbox 0 */
#define CAN_ID_CMD       0x180U        /* 上→下 速度/模式/使能 50Hz */
#define CAN_ID_PARAM     0x181U        /* 上→下 参数请求 */
#define CAN_ID_ENC       0x280U        /* 下→上 编码器累计计数 100Hz */
#define CAN_ID_PRM_ACK   0x282U        /* 下→上 参数应答 */
#define CAN_ID_STATUS    0x300U        /* 下→上 状态 10Hz */
#define CAN_DLC          8U

/* 速度换算（实物标定）：13PPR×4×22.569=1173.588 counts/轮转，轮径65mm 周长204.20mm
 * 5.747 counts/mm × 0.5 = 2.873 单位/(mm/s) → mm/s × 2873/1000 = 每500ms计数 */
#define MM_TO_CNT_Q      (2873)    /* mm/s → 每500ms计数 定点系数（/1000） */
#define TRACK_W_MM_Q     (1214)    /* 轮距 121.4mm（×10 定点） */

/* 0x180 解析结果（阶段1只存储，未接进电机；mm/s→现有速度单位换算待确认） */
static volatile int16_t  g_can_v_ref = 0;      /* mm/s 线速度 */
static volatile int16_t  g_can_w_ref = 0;      /* mrad/s 角速度 */
static volatile uint8_t  g_can_mode  = 0;      /* 0 IDLE 1 TELEOP 2 AUTO */
static volatile uint8_t  g_can_flags = 0;      /* bit0 enable, bit1 clear_fault, bit2 soft_estop */
static volatile uint8_t  g_can_seq   = 0;      /* 命令序号 */
static volatile uint32_t g_can_age_ms = 0;     /* 距最后有效 0x180（ms） */
static uint8_t g_can_boot_id = 1;              /* 每次复位 +1 */
static uint8_t g_can_tx_seq = 0;               /* 发送帧 seq 递增 */
static volatile uint32_t g_tick_ms;            /* 前向声明（定义在下方） */

/* mm/s → 现有速度单位（每500ms计数） */
static int32_t mm_s_to_units(int32_t mm_s)
{
    return (int32_t)(((int64_t) mm_s * MM_TO_CNT_Q) / 1000);
}

/* 发送一帧经典 CAN；buffer: mailbox 号或 TX FIFO（CAN_MAILBOX_ID_TX_FIFO） */
static void can_send(uint32_t buffer, uint32_t id, const uint8_t *data)
{
    can_frame_t f;
    f.id               = id;
    f.id_mode          = CAN_ID_MODE_STANDARD;
    f.type             = CAN_FRAME_TYPE_DATA;
    f.data_length_code = CAN_DLC;
    f.options          = 0;
    for (uint8_t i = 0; i < CAN_DLC; i++) { f.data[i] = data[i]; }
    g_can0.p_api->write(g_can0.p_ctrl, buffer, &f);
}

/* 0x280：编码器累计计数 + 采样时刻 + 状态 + seq（100Hz） */
static void can_send_280(uint32_t t_ms, int16_t cnt_l, int16_t cnt_r, uint8_t status)
{
    uint8_t d[CAN_DLC];
    d[0] = (uint8_t)(t_ms & 0xFF);        d[1] = (uint8_t)((t_ms >> 8) & 0xFF);
    d[2] = (uint8_t)(cnt_l & 0xFF);       d[3] = (uint8_t)((cnt_l >> 8) & 0xFF);
    d[4] = (uint8_t)(cnt_r & 0xFF);       d[5] = (uint8_t)((cnt_r >> 8) & 0xFF);
    d[6] = status;
    d[7] = g_can_tx_seq++;
    can_send(CAN_TX_MB, CAN_ID_ENC, d);          /* 0x280 走 mailbox 0 */
}

/* 0x300：电压/温度/故障/复位/总线（10Hz） */
static void can_send_300(uint16_t vbat_mv, int8_t temp_c, uint8_t err, uint8_t can_err)
{
    uint8_t d[CAN_DLC];
    d[0] = (uint8_t)(vbat_mv & 0xFF);     d[1] = (uint8_t)((vbat_mv >> 8) & 0xFF);
    d[2] = (uint8_t) temp_c;
    d[3] = err;
    d[4] = g_can_boot_id;
    d[5] = can_err;
    d[6] = (uint8_t)(g_can_age_ms & 0xFF);
    d[7] = (uint8_t)((g_can_age_ms >> 8) & 0xFF);
    can_send(CAN_MAILBOX_ID_TX_FIFO, CAN_ID_STATUS, d);   /* 0x300 走 TX FIFO，避免与 0x280 抢 mailbox */
}

/* CAN 接收回调（r_can 驱动纯中断接收，帧在 p_args->frame） */
void can_callback(can_callback_args_t *p_args)
{
    if (0 == (p_args->event & CAN_EVENT_RX_COMPLETE)) { return; }
    can_frame_t *f = &p_args->frame;
    uint16_t id = (uint16_t)(f->id & 0x7FFU);

    /* CAN 收帧转发到串口（有线 COM4 观察）；中断里 sprintf+异步write 对 50Hz 足够 */
    {
        char b[48];
        uint32_t n = (uint32_t) sprintf(b, "CAN RX %03X: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                          (unsigned) id,
                          f->data[0], f->data[1], f->data[2], f->data[3],
                          f->data[4], f->data[5], f->data[6], f->data[7]);
        g_uart0.p_api->write(g_uart0.p_ctrl, (uint8_t *)b, n);
    }

    if (CAN_ID_CMD == id)
    {
        /* 0x180：v_ref(mm/s)/w_ref(mrad/s)/mode/flags/seq（小端） */
        int16_t v = (int16_t)(f->data[0] | ((uint16_t) f->data[1] << 8));
        int16_t w = (int16_t)(f->data[2] | ((uint16_t) f->data[3] << 8));
        g_can_v_ref = v; g_can_w_ref = w;
        g_can_mode  = f->data[4];
        g_can_flags = f->data[5];
        g_can_seq   = f->data[6];
        g_can_age_ms = 0;
        g_last_cmd_ms = g_tick_ms;          /* 与串口共用 200ms 超时停车 */

        if (g_can_flags & 0x01U)            /* enable=1 → 应用速度 */
        {
            /* v_l = v - w*b/2000, v_r = v + w*b/2000（b=121.4mm, w 单位 mrad/s） */
            int32_t dv = (int32_t)(((int64_t) w * TRACK_W_MM_Q) / 20000);
            g_left_target  = mm_s_to_units((int32_t) v - dv);
            g_right_target = mm_s_to_units((int32_t) v + dv);
        }
        else                                /* enable=0 → 停车 */
        {
            g_left_target  = 0;
            g_right_target = 0;
        }
    }
    /* 0x181 参数请求：阶段2实现（0x282 应答） */
}

/* === 系统 tick（ms） === */
static volatile uint32_t g_tick_ms = 0;

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

/* === 解析命令（CAN 联调阶段：串口电机运动控制已暂时禁用） === */
/* L/R/T 不再驱动电机；仅保留 Kp:/Ki:/Kd: 调参（共用参数），便于联调微调 */
/* 运动目标仅由 CAN 0x180 设置 */

/* 回显同时发往有线(SCI9)和无线(SCI0)，方便无线链路验证 */
static void ack_both(const char *s, uint32_t n)
{
    g_uart0.p_api->write(g_uart0.p_ctrl, (uint8_t *)s, n);
    g_uart1.p_api->write(g_uart1.p_ctrl, (uint8_t *)s, n);
}

/*串口指令*/
static void parse_rx_line(const char *line)
{
    int32_t v = 0;
    char ack[40];

    if (1 == sscanf(line, "Kp:%d", &v))
    {
        if (v >= 1 && v <= 500) { g_kp_int = v; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK Kp:%d (=%.2f)\r\n", g_kp_int, (float)g_kp_int / 100.0f);
        ack_both(ack, n);
    }
    else if (1 == sscanf(line, "Ki:%d", &v))
    {
        if (v >= 0 && v <= 200) { g_ki_int = v; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK Ki:%d (=%.2f)\r\n", g_ki_int, (float)g_ki_int / 100.0f);
        ack_both(ack, n);
    }
    else if (1 == sscanf(line, "Kd:%d", &v))
    {
        if (v >= 0 && v <= 200) { g_kd_int = v; }
        g_last_cmd_ms = g_tick_ms;
        pid_reset_state();
        uint32_t n = (uint32_t) sprintf(ack, "OK Kd:%d (=%.2f)\r\n", g_kd_int, (float)g_kd_int / 100.0f);
        ack_both(ack, n);
    }
    /* T:%d 原地旋转指令已随串口电机控制一起暂时禁用 */
}

/* 从通道取出一条完整命令：临界区拷贝到本地再解析，避免与中断写竞争 */
static void service_rx(rx_chan_t *ch)
{
    char    line[64];
    uint8_t len = 0, ready = 0;

    __disable_irq();
    len   = ch->idx;
    ready = ch->line_ready;
    if (ready || (len > 0 && (g_tick_ms - ch->last_char_ms) >= RX_PARSE_IDLE_MS))
    {
        for (uint8_t i = 0; i < len; i++) line[i] = ch->buf[i];
        line[len] = '\0';
        ch->idx = 0;
        ch->line_ready = 0;
    }
    __enable_irq();

    if (len) parse_rx_line(line);
}

/* === 串口回调：逐字节累积到缓冲 === */
/* user_uart_callback  = SCI9 (P109/P110, 有线调试) */
/* wireless_uart_callback = SCI0 (P100/P101, MX-02P 无线) */
/* 两个串口的 RX 各自累积到独立缓冲，共用同一套命令解析 */
/* 单通道字符累积：有线/无线各自独立缓冲，互不覆盖 */
static void rx_push(rx_chan_t *ch, char c)
{
    if (c == '\r' || c == '\n')
    {
        ch->line_ready = 1;   /* 回车 → 整条命令就绪（蓝牙分段也不拆开） */
    }
    else if (ch->idx < sizeof(ch->buf) - 1)
    {
        ch->buf[ch->idx++] = c;
        ch->last_char_ms = g_tick_ms;
        ch->line_ready = 0;   /* 有新内容，等待真正结束 */
    }
}

void user_uart_callback(uart_callback_args_t *p_args)
{
    if (UART_EVENT_RX_CHAR == p_args->event)
        rx_push(&g_rx_uart0, (char) p_args->data);
}

void wireless_uart_callback(uart_callback_args_t *p_args)
{
    if (UART_EVENT_RX_CHAR == p_args->event)
        rx_push(&g_rx_uart1, (char) p_args->data);
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

    /* CAN 初始化（r_can 经典帧 500kbps, P401=CTX0/P402=CRX0） */
    err = g_can0.p_api->open(g_can0.p_ctrl, g_can0.p_cfg);
    if (FSP_SUCCESS == err)
    {
        /* 启动自测：内部回环发一帧，验证接收中断+回调+串口转发。
         * 若 COM4 启动打印 "CAN RX 280: ..."，说明下位机接收链路全通。 */
        g_can0.p_api->modeTransition(g_can0.p_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_LOOPBACK_INTERNAL);
        can_send_280(1234U, 0, 0, 0);
        R_BSP_SoftwareDelay(30U, BSP_DELAY_UNITS_MILLISECONDS);
        g_can0.p_api->modeTransition(g_can0.p_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);
    }

    int32_t prev_l = 0, prev_r = 0;   /* 编码器上次计数值（差分） */
    uint32_t last_pid_ms = 0;
    uint32_t last_can280_ms = 0, last_can300_ms = 0;

    while(1)
    {
        g_tick_ms += 10;

        /* CAN 命令老化计时：距最后有效 0x180 */
        g_can_age_ms += 10;

        /* 串口命令解析：有线/无线各自独立缓冲，收到回车即整条解析；空闲超时兜底 */
        service_rx(&g_rx_uart0);
        service_rx(&g_rx_uart1);

        /* 每 10ms 发 0x280（100Hz）：编码器累计计数 + 采样时刻 */
        if (g_tick_ms - last_can280_ms >= 10U)
        {
            last_can280_ms = g_tick_ms;
            /* 方向与现有控制一致：左轮取反，右轮原样 */
            int16_t cnt_l = (int16_t)(-(int32_t) R_GPT3->GTCNT);
            int16_t cnt_r = (int16_t)( (int32_t) R_GPT0->GTCNT);
            can_send_280(g_tick_ms, cnt_l, cnt_r, 0);   /* status 阶段1暂 0 */
        }

        /* 每 100ms 发 0x300（10Hz）：电压/温度/故障/总线 */
        if (g_tick_ms - last_can300_ms >= 100U)
        {
            last_can300_ms = g_tick_ms;
            can_send_300(12000, 25, 0, 0);   /* vbat=12.0V/temp=25°C 占位, err=0, can_err=0 */
        }

        /* PID 更新 */
        if (g_tick_ms - last_pid_ms >= PID_UPDATE_MS)
        {
            last_pid_ms = g_tick_ms;

            /* 接收超时停车：200ms 无任何串口指令 → 自动停车 */
            /* 遥控场景下，松开方向键后 MCU 不再收指令，会自动停（不依赖停车指令传不传得到） */
            if (g_tick_ms - g_last_cmd_ms > RX_TIMEOUT_MS)
            {
                g_left_target  = 0;
                g_right_target = 0;
            }

            int32_t enc_l = (int32_t) R_GPT3->GTCNT;   /* 左轮累计计数 */
            int32_t enc_r = (int32_t) R_GPT0->GTCNT;   /* 右轮累计计数 */
            /* 差分换算成"每500ms"速度；L 侧反转使正转=正速度 */
            int32_t raw_l = (prev_l - enc_l) * (int32_t) SPEED_SCALE;
            int32_t raw_r = (enc_r - prev_r) * (int32_t) SPEED_SCALE;
            prev_l = enc_l;
            prev_r = enc_r;
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

        R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

#if BSP_TZ_SECURE_BUILD
FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable () { }
FSP_CPP_FOOTER
#endif
