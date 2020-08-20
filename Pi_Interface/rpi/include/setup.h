void print_received_message(void);
void process_message(void);
void send_light_request(void);
void request_data(void);
void request_all_data(void);


void set_packet_size(uint8_t*, uint8_t);
void set_source_address(uint8_t*, uint16_t);
void set_target_node(uint8_t*,uint16_t);
void set_final_node(uint8_t*, uint16_t);
void set_command(uint8_t*, uint16_t, uint8_t, uint8_t, uint8_t, uint8_t);
void set_request_id(uint8_t*);
void set_ADC_channel(uint8_t*, uint8_t);

uint16_t get_source_node_address(uint8_t*);
uint8_t get_message_size(uint8_t*);
uint16_t get_command(uint8_t*);
uint8_t get_command_data(uint8_t*, uint8_t);
uint16_t get_target_node(uint8_t*);
uint16_t get_final_node(uint8_t*);

void print_message(uint8_t *);
void process_message();

void request_input(void);

void send_command(uint16_t, uint8_t*);
void send_ping(uint16_t, uint8_t*);
void send_setup(uint16_t, uint8_t*);
void send_setup_request(void);
void send_request_all_data(uint8_t);

void send_set_light(uint16_t, uint16_t,  uint8_t);
void send_request_data(uint16_t, uint16_t, uint8_t);

void continue_data_request(uint8_t,uint16_t);

void process_message();
void record_message(uint8_t*);

