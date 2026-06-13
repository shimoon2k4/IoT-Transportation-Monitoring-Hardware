#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_system.h"

#if defined( REGION_AS923 )
#define RF_FREQUENCY                                923000000
#elif defined( REGION_AU915 )
#define RF_FREQUENCY                                915000000
#elif defined( REGION_CN470 )
#define RF_FREQUENCY                                470000000
#elif defined( REGION_CN779 )
#define RF_FREQUENCY                                779000000
#elif defined( REGION_EU433 )
#define RF_FREQUENCY                                433000000
#elif defined( REGION_EU868 )
#define RF_FREQUENCY                                868000000
#elif defined( REGION_KR920 )
#define RF_FREQUENCY                                920000000
#elif defined( REGION_IN865 )
#define RF_FREQUENCY                                865000000
#elif defined( REGION_US915 )
#define RF_FREQUENCY                                915000000
#elif defined( REGION_US915_HYBRID )
#define RF_FREQUENCY                                915000000
#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14

#if defined( USE_MODEM_LORA )
#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#elif defined( USE_MODEM_FSK )
#define FSK_FDEV                                    25000
#define FSK_DATARATE                                50000
#define FSK_BANDWIDTH                               50000
#define FSK_AFC_BANDWIDTH                           83333
#define FSK_PREAMBLE_LENGTH                         5
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum { LOWPOWER, RX, RX_TIMEOUT, RX_ERROR, TX, TX_TIMEOUT } States_t;

#define RX_TIMEOUT_VALUE        3000
#define BUFFER_SIZE             280
#define CHUNK_MAX               270

#define MAX_JUMP_THRESHOLD          20000  // Cho phép gap lớn hơn trong mạng yếu
#define ANTI_REPLAY_WINDOW_SEC      300
#define CLOCK_SKEW_TOLERANCE_SEC    5
#define MAX_NODES                   10
#define WINDOW_SIZE 32  // Khớp với 32-bit bitmap (uint32_t)

#define SECRET_KEY_LEN 16
static const uint8_t SECRET_KEY[SECRET_KEY_LEN] = {
    0x13,0x37,0xAA,0x55,0x99,0x42,0xDE,0xAD,
    0xBE,0xEF,0x77,0x21,0x66,0x88,0xCC,0x10
};

static uint16_t compute_auth_tag(uint8_t node_id, uint32_t seq, const uint8_t* payload, uint16_t len) {
    uint8_t auth_buf[BUFFER_SIZE];
    int p = 0;
    memcpy(&auth_buf[p], SECRET_KEY, SECRET_KEY_LEN); p += SECRET_KEY_LEN;
    auth_buf[p++] = node_id;
    auth_buf[p++] = (seq >> 0) & 0xFF;
    auth_buf[p++] = (seq >> 8) & 0xFF;
    auth_buf[p++] = (seq >> 16) & 0xFF;
    auth_buf[p++] = (seq >> 24) & 0xFF;
    memcpy(&auth_buf[p], payload, len); p += len;
    return crc16_calc(auth_buf, p);
}

uint8_t Buffer[BUFFER_SIZE];
uint16_t BufferSize = 0;

static uint8_t  LoraBuf[BUFFER_SIZE];
static uint16_t LoraLen = 0;

volatile States_t State = LOWPOWER;
int8_t RssiValue = 0;
int8_t SnrValue = 0;
uint32_t ChipId[2] = {0};

static RadioEvents_t RadioEvents;

static uint32_t valid_packets = 0;
static uint32_t invalid_seq = 0;
static uint32_t invalid_crc = 0;
static uint32_t invalid_jump = 0;

// === DEMO 3: Timing Tracking ===
static uint32_t demo_start_time = 0;
static uint32_t last_rx_time[MAX_NODES];
#define TIMED_LOG(fmt, ...) do { \
    uint32_t elapsed = TimerGetCurrentTime() - demo_start_time; \
    printf("[%7lu ms] " fmt "\r\n", (unsigned long)elapsed, ##__VA_ARGS__); \
} while(0)

typedef struct {
    uint8_t  node_id;
    uint32_t last_seq;
    uint32_t bitmap;
    uint32_t packets_received;
    uint32_t packets_rejected;
    int8_t   last_rssi;
} PerNodeState_t;

typedef struct {
    uint8_t  node_id;
    uint32_t seq;
    uint16_t payload_len;
    uint8_t  payload[CHUNK_MAX];
    uint16_t crc;
    int      valid;
    char     reject_reason[80];
} ParsedPacket_t;

static PerNodeState_t node_states[MAX_NODES];
static uint8_t node_count = 0;

void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);

static uint16_t crc16_calc(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
            crc &= 0xFFFF;
        }
    }
    return crc;
}

static void per_node_init(void)
{
    memset(node_states, 0, sizeof(node_states));
    node_count = 0;
}

static PerNodeState_t* per_node_get_or_create(uint8_t node_id)
{
    for (uint8_t i = 0; i < node_count; i++) {
        if (node_states[i].node_id == node_id) {
            return &node_states[i];
        }
    }

    if (node_count < MAX_NODES) {
        node_states[node_count].node_id = node_id;
        node_states[node_count].last_seq = 0;
        node_states[node_count].packets_received = 0;
        node_states[node_count].packets_rejected = 0;
        node_states[node_count].last_rssi = 0;
        return &node_states[node_count++];
    }

    return NULL;
}

static int check_and_update_seq(PerNodeState_t* t, uint32_t seq)
{
    if (seq > t->last_seq) {
        uint32_t shift = seq - t->last_seq;

        if (shift > WINDOW_SIZE) {
            printf("[DROP] Jump too large: %lu\r\n", (unsigned long)shift);
            return 0;
        }

        if (shift >= WINDOW_SIZE) {
            t->bitmap = 0;
        } else {
            t->bitmap <<= shift;
        }

        t->bitmap |= 1;
        t->last_seq = seq;
        return 1;
    }

    uint32_t diff = t->last_seq - seq;

    if (diff >= WINDOW_SIZE) {
        printf("[DROP] Too old\r\n");
        return 0;
    }

    if (t->bitmap & (1UL << diff)) {
        printf("[DROP] Replay\r\n");
        return 0;
    }

    t->bitmap |= (1UL << diff);
    return 1;
}

static void parse_and_validate_packet(const uint8_t* raw, uint16_t raw_len, ParsedPacket_t* pkt)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->valid = 0;

    if (raw_len < 9) {
        snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Packet quá nhỏ (%u)", raw_len);
        return;
    }

    int pos = 0;
    pkt->node_id = raw[pos++];

    pkt->seq = (uint32_t)raw[pos+0]
             | ((uint32_t)raw[pos+1] << 8)
             | ((uint32_t)raw[pos+2] << 16)
             | ((uint32_t)raw[pos+3] << 24);
    pos += 4;

    pkt->payload_len = (uint16_t)raw[pos+0] | ((uint16_t)raw[pos+1] << 8);
    pos += 2;

    if (pkt->payload_len > CHUNK_MAX || pos + pkt->payload_len + 2 > raw_len) {
        snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Payload len không hợp lệ");
        return;
    }

    memcpy(pkt->payload, &raw[pos], pkt->payload_len);
    pos += pkt->payload_len;
    pkt->crc = (uint16_t)raw[pos+0] | ((uint16_t)raw[pos+1] << 8);

    // BƯỚC 1: Kiểm tra AUTH TAG (Giữ nguyên như yêu cầu)
    {
        uint16_t expected = compute_auth_tag(pkt->node_id, pkt->seq, pkt->payload, pkt->payload_len);
        if (expected != pkt->crc) {
            snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "AUTH FAIL");
            return;
        }
    }

    // BƯỚC 2: Kiểm tra Anti-Replay bằng Sliding Window Bitmap
    {
        PerNodeState_t* node = per_node_get_or_create(pkt->node_id);
        if (!node) {
            snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Max nodes exceeded");
            return;
        }

        // Node mới (first packet)
        if (node->packets_received == 0) {

            // (optional) chống attacker gửi seq cực lớn ngay từ đầu
            if (pkt->seq > 1000000) {
                snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "First seq too large");
                return;
            }

            node->last_seq = pkt->seq;
            node->bitmap   = 1;
            pkt->valid     = 1;
        }
        else {
            if (check_and_update_seq(node, pkt->seq)) {
                pkt->valid = 1;
            } else {
                snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Replay/Too old");
                node->packets_rejected++;
                return;
            }
        }

        // Cập nhật thống kê nếu gói hợp lệ
        if (pkt->valid) {
            node->last_rssi = (int8_t)RssiValue;
            node->packets_received++;
        }
    }
}

int app_start(void)
{
    (void)system_get_chip_id(ChipId);
    printf("Gateway Chip ID: 0x%08lX-%08lX\r\n",
           (unsigned long)ChipId[0],
           (unsigned long)ChipId[1]);
    printf("\r\n");

    per_node_init();
    memset(last_rx_time, 0, sizeof(last_rx_time));
    demo_start_time = TimerGetCurrentTime();

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

#if defined( USE_MODEM_LORA )
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
#elif defined( USE_MODEM_FSK )
    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);
#endif

    Radio.Rx(RX_TIMEOUT_VALUE);

    printf("Listening for Follower Nodes...\r\n");
    printf("Max nodes: %u | Max payload: %u bytes\r\n", MAX_NODES, CHUNK_MAX);
    printf("==============================================\r\n\r\n");

    static uint32_t total_accepted = 0;
    static uint32_t total_rejected = 0;

    while (1)
    {
        switch (State)
        {
        case RX:
            {
                ParsedPacket_t pkt;
                parse_and_validate_packet(LoraBuf, LoraLen, &pkt);

                if (pkt.valid) {
                    total_accepted++;
                    uint32_t now_ms = TimerGetCurrentTime();
                    uint32_t delta_ms = (last_rx_time[pkt.node_id] > 0) ? 
                        (now_ms - last_rx_time[pkt.node_id]) : 0;
                    last_rx_time[pkt.node_id] = now_ms;
                    
                    TIMED_LOG("[RX OK] node=%u, seq=%lu, len=%u, delta=%lu ms, rssi=%d dBm, snr=%d dB",
                           (unsigned)pkt.node_id,
                           (unsigned long)pkt.seq,
                           (unsigned)pkt.payload_len,
                           (unsigned long)delta_ms,
                           (int)RssiValue,
                           (int)SnrValue);

                    if (pkt.payload_len > 0) {
                        char payload_str[CHUNK_MAX + 1];
                        memcpy(payload_str, (char*)pkt.payload, pkt.payload_len);
                        payload_str[pkt.payload_len] = '\0';
                        printf("        Payload: %s\r\n", payload_str);
                    }
                } else {
                    total_rejected++;
                    TIMED_LOG("[RX DROP] node=%u, seq=%lu, rssi=%d (Reason: %s)",
                           (unsigned)pkt.node_id,
                           (unsigned long)pkt.seq,
                           (int)RssiValue,
                           pkt.reject_reason);
                }
            }
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case TX:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            break;
        }

        Radio.IrqProcess();

        {
            static uint32_t last_stats = 0;
            uint32_t now = TimerGetCurrentTime();
            if (TimerGetElapsedTime(last_stats) > 30000) {
                printf("\r\n");
                TIMED_LOG("=== GATEWAY STATISTICS (30sec) ===");
                TIMED_LOG("Total accepted: %lu | Total rejected: %lu", 
                    (unsigned long)total_accepted, (unsigned long)total_rejected);
                TIMED_LOG("Active nodes:   %u/%u", node_count, MAX_NODES);
                printf("\r\n");

                TIMED_LOG("Per-Node Status:");
                for (uint8_t i = 0; i < node_count; i++) {
                    uint32_t total = node_states[i].packets_received + node_states[i].packets_rejected;
                    uint32_t rate = (total > 0) ? (100U * node_states[i].packets_received / total) : 0;
                    TIMED_LOG("  Node %u: seq=%lu, ok=%lu, bad=%lu, rate=%lu%%, rssi=%d dBm",
                           (unsigned)node_states[i].node_id,
                           (unsigned long)node_states[i].last_seq,
                           (unsigned long)node_states[i].packets_received,
                           (unsigned long)node_states[i].packets_rejected,
                           (unsigned long)rate,
                           (int)node_states[i].last_rssi);
                }
                TIMED_LOG("==================================\r\n");

                last_stats = now;
            }
        }
    }
}

void OnTxDone(void)
{
    Radio.Sleep();
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();

    uint16_t use_len = (size > BUFFER_SIZE) ? BUFFER_SIZE : size;
    memcpy(LoraBuf, payload, use_len);
    LoraLen = use_len;
    RssiValue = rssi;
    SnrValue = snr;

    State = RX;
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    Radio.Sleep();
    State = RX_ERROR;
}
