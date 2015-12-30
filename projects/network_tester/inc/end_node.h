#ifndef __END_NODE_H__
#define __END_NODE_H__


void enrollment_state_get(uint8_t* p_state);
void enrollment_state_set(uint8_t state);

uint32_t network_token_allowed_get(void);
uint32_t network_token_blocked_get(void);

// Consume packets that came down from gateway

struct packet_queue;
struct llabs_frame_header;
struct llabs_info_block0;
struct llabs_info_block1;

void end_node_iterate(struct packet_queue* q_in,
                      struct packet_queue* q_out,
                      struct llabs_frame_header* p_fh,
                      struct llabs_info_block0* p_ib0,
                      struct llabs_info_block0* p_ib1,
                      void* tbd);

#endif //__END_NODE_H__
