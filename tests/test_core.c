#include "fc_core.h"
#include "whoopy_radio_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void){
 whoopy_control_t in={.type=WHOOPY_PACKET_CONTROL,.version=WHOOPY_PROTO_VERSION,.sequence=65535,.roll=500,.pitch=0,.yaw=1000,.throttle=123,.flags=WHOOPY_FLAG_ARM,.sender_ms=0x12345678};
 uint8_t bytes[WHOOPY_PACKET_SIZE];assert(whoopy_encode_control(&in,bytes));assert(sizeof bytes==32);assert(bytes[14]==0x78&&bytes[17]==0x12);
 whoopy_control_t out={0};assert(whoopy_decode_control(bytes,sizeof bytes,&out));assert(out.sequence==65535&&out.pitch==0&&out.yaw==1000&&out.sender_ms==in.sender_ms);
 bytes[6]^=1;assert(!whoopy_decode_control(bytes,sizeof bytes,&out));bytes[6]^=1;assert(!whoopy_decode_control(bytes,31,&out));bytes[1]=2;assert(!whoopy_decode_control(bytes,sizeof bytes,&out));
 in.roll=1001;assert(!whoopy_encode_control(&in,bytes));in.roll=500;in.flags=WHOOPY_FLAG_ARM|WHOOPY_FLAG_DISARM;assert(!whoopy_encode_control(&in,bytes));in.flags=WHOOPY_FLAG_ARM;
 assert(whoopy_seq_newer(0,65535));assert(!whoopy_seq_newer(65535,0));assert(!whoopy_seq_newer(10,10));
 whoopy_pid_cfg_t pc={.kp=1,.ki=0,.kd=0,.limit=.5f,.integral_limit=1};whoopy_pid_state_t ps={0};assert(fabsf(whoopy_pid_step(&ps,&pc,1,0,.004f)-.5f)<.0001f);assert(whoopy_pid_step(&ps,&pc,0,0,.004f)==0);
 whoopy_mix_t m;whoopy_mix_x(.5f,.1f,.1f,.05f,&m);assert(fabsf(m.motor[0]-.65f)<1e-6f);for(int i=0;i<4;i++)assert(m.motor[i]>=0&&m.motor[i]<=1);whoopy_mix_x(.9f,1,1,1,&m);for(int i=0;i<4;i++)assert(m.motor[i]>=0&&m.motor[i]<=1);
 assert(whoopy_config_valid(&pc,20,180,250));pc.kp=NAN;assert(!whoopy_config_valid(&pc,20,180,250));pc.kp=1;assert(!whoopy_config_valid(&pc,60,180,250));assert(!whoopy_config_valid(&pc,20,180,50));
 whoopy_flight_state_t s=WHOOPY_BOOT;s=whoopy_safety_step(s,true,true,true,false,false,true);assert(s==WHOOPY_INITIALIZING);s=whoopy_safety_step(s,true,true,true,false,false,true);assert(s==WHOOPY_DISARMED);s=whoopy_safety_step(s,true,true,true,true,false,true);assert(s==WHOOPY_ARMED);s=whoopy_safety_step(s,true,true,false,false,false,true);assert(s==WHOOPY_FAILSAFE);s=whoopy_safety_step(s,true,true,true,true,false,true);assert(s==WHOOPY_DISARMED);s=whoopy_safety_step(s,true,true,true,false,false,true);assert(s==WHOOPY_DISARMED);
 puts("core tests passed");return 0;
}
