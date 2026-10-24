/* Host mocks for os_port and motor, used only by the PC unit tests. */
#include <string.h>
#include "s2_os_port.h"
#include "drv_motor.h"

#define MOCK_MAX_OBJ   (16)
#define MOCK_MBF_BYTES (512)

uint32_t g_mock_now_ms = 0u;
uint32_t g_mock_flags[MOCK_MAX_OBJ];
static os_id_t s_next_id = 1;

typedef struct
{
    uint8_t  buf[MOCK_MBF_BYTES];
    uint32_t sizes[64];
    uint32_t head, tail, count, bytes_used, max_msg, buf_bytes;
    uint32_t off_head, off_tail;
} mock_mbf_t;
static mock_mbf_t s_mbf[MOCK_MAX_OBJ];

os_id_t os_flag_create(uint32_t init);
os_id_t os_mbf_create(uint32_t buf_bytes, uint32_t max_msg);
os_id_t os_flag_create(uint32_t init) { os_id_t id = s_next_id++; g_mock_flags[id] = init; return id; }
os_id_t os_mbf_create(uint32_t buf_bytes, uint32_t max_msg)
{
    os_id_t id = s_next_id++;
    memset(&s_mbf[id], 0, sizeof(s_mbf[id]));
    s_mbf[id].max_msg = max_msg; s_mbf[id].buf_bytes = buf_bytes;
    return id;
}

void os_flag_set(os_id_t f, uint32_t b)   { g_mock_flags[f] |= b; }
void os_flag_clear(os_id_t f, uint32_t b) { g_mock_flags[f] &= ~b; }
bool os_flag_poll(os_id_t f, uint32_t b)  { return (g_mock_flags[f] & b) != 0u; }
bool os_flag_wait_any(os_id_t f, uint32_t b, uint32_t t) { (void)t; return (g_mock_flags[f] & b) != 0u; }

/* Simple FIFO; messages stored in fixed max_msg slots */
bool os_mbf_send(os_id_t id, const void *p, uint32_t size, int32_t t)
{
    mock_mbf_t *m = &s_mbf[id];
    (void)t;
    if ((size > m->max_msg) || (m->count >= 64u) || ((m->count + 1u) * m->max_msg > m->buf_bytes)) { return false; }
    memcpy(&m->buf[(m->tail % 64u) * m->max_msg % MOCK_MBF_BYTES], p, size);
    m->sizes[m->tail % 64u] = size;
    m->tail++; m->count++;
    return true;
}
int32_t os_mbf_recv_poll(os_id_t id, void *p)
{
    mock_mbf_t *m = &s_mbf[id];
    uint32_t n;
    if (m->count == 0u) { return -1; }
    n = m->sizes[m->head % 64u];
    memcpy(p, &m->buf[(m->head % 64u) * m->max_msg % MOCK_MBF_BYTES], n);
    m->head++; m->count--;
    return (int32_t)n;
}

uint32_t os_now_ms(void) { return g_mock_now_ms; }
void     os_sleep_ms(uint32_t ms) { g_mock_now_ms += ms; }

/* Motor mock */
int32_t g_mock_duty[2];
int     g_mock_stop_all_calls = 0;
void motor_init(void) { g_mock_duty[0] = 0; g_mock_duty[1] = 0; }
void motor_set(motor_side_t s, int32_t d) { g_mock_duty[s] = d; }
void motor_stop_all(void) { g_mock_duty[0] = 0; g_mock_duty[1] = 0; g_mock_stop_all_calls++; }
void motion_debug_pin(bool high) { (void)high; }
