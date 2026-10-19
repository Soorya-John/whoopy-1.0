#ifndef WHOOPY_RADIO_PROTOCOL_H
#define WHOOPY_RADIO_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Wire encoding is little-endian, fixed offsets, exactly 32 bytes. No struct ABI. */
#define WHOOPY_PROTO_VERSION 1u
#define WHOOPY_PACKET_CONTROL 1u
#define WHOOPY_PACKET_ACK 2u
#define WHOOPY_PACKET_SIZE 32u
#define WHOOPY_RADIO_CHANNEL 76u
#define WHOOPY_RADIO_RATE "1Mbps"
#define WHOOPY_RADIO_ADDRESS "WHOPY"
#define WHOOPY_FLAG_ARM 0x01u
#define WHOOPY_FLAG_DISARM 0x02u
#define WHOOPY_FLAG_ACRO 0x04u
#define WHOOPY_FLAG_MASK 0x07u
#define WHOOPY_AXIS_COUNT 4u
#define WHOOPY_PAYLOAD_MIN 0u
#define WHOOPY_PAYLOAD_MAX 1000u

typedef struct {
    uint8_t type, version;
    uint16_t sequence;
    uint16_t roll, pitch, yaw, throttle; /* 0..1000; centered at 500 except throttle */
    uint8_t flags;
    uint32_t sender_ms;
} whoopy_control_t;

static inline uint16_t whoopy_get_u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t whoopy_get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static inline void whoopy_put_u16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void whoopy_put_u32(uint8_t *p,uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static inline uint16_t whoopy_crc16(const uint8_t *p,size_t n) {
    uint16_t crc=0xffffu; for(size_t i=0;i<n;i++){ crc^=(uint16_t)p[i]<<8; for(int b=0;b<8;b++) crc=(crc&0x8000u)?(uint16_t)((crc<<1)^0x1021u):(uint16_t)(crc<<1); } return crc;
}
static inline bool whoopy_encode_control(const whoopy_control_t *c,uint8_t out[WHOOPY_PACKET_SIZE]) {
    if(!c||!out||c->type!=WHOOPY_PACKET_CONTROL||c->version!=WHOOPY_PROTO_VERSION||c->flags&~WHOOPY_FLAG_MASK||((c->flags&(WHOOPY_FLAG_ARM|WHOOPY_FLAG_DISARM))==(WHOOPY_FLAG_ARM|WHOOPY_FLAG_DISARM))||c->roll>1000||c->pitch>1000||c->yaw>1000||c->throttle>1000) return false;
    out[0]=c->type; out[1]=c->version; whoopy_put_u16(out+2,c->sequence); whoopy_put_u16(out+4,c->roll); whoopy_put_u16(out+6,c->pitch); whoopy_put_u16(out+8,c->yaw); whoopy_put_u16(out+10,c->throttle); out[12]=c->flags; out[13]=0; whoopy_put_u32(out+14,c->sender_ms); for(int i=18;i<30;i++)out[i]=0; whoopy_put_u16(out+30,whoopy_crc16(out,30)); return true;
}
static inline bool whoopy_decode_control(const uint8_t *p,size_t n,whoopy_control_t *c) {
    if(!p||!c||n!=WHOOPY_PACKET_SIZE||p[0]!=WHOOPY_PACKET_CONTROL||p[1]!=WHOOPY_PROTO_VERSION||p[12]&~WHOOPY_FLAG_MASK||((p[12]&(WHOOPY_FLAG_ARM|WHOOPY_FLAG_DISARM))==(WHOOPY_FLAG_ARM|WHOOPY_FLAG_DISARM))||p[13]!=0) return false;
    for(int i=18;i<30;i++)if(p[i]!=0)return false;
    if(whoopy_get_u16(p+30)!=whoopy_crc16(p,30))return false;
    c->type=p[0]; c->version=p[1]; c->sequence=whoopy_get_u16(p+2); c->roll=whoopy_get_u16(p+4); c->pitch=whoopy_get_u16(p+6); c->yaw=whoopy_get_u16(p+8); c->throttle=whoopy_get_u16(p+10); c->flags=p[12]; c->sender_ms=whoopy_get_u32(p+14);
    return c->roll<=1000&&c->pitch<=1000&&c->yaw<=1000&&c->throttle<=1000;
}
static inline bool whoopy_seq_newer(uint16_t a,uint16_t b) { return a!=b&&(uint16_t)(a-b)<0x8000u; }
#endif
