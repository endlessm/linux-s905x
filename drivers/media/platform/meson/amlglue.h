#include "../../../amlogic/amports/streambuf.h"
#include "../../../amlogic/amports/esparser.h"
#include "../../../amlogic/amports/amports_priv.h"
#include "../../../amlogic/amports/vdec.h"

#define PARSER_VIDEO        (ES_TYPE_VIDEO)

extern struct dec_sysinfo amstream_dec_info;

stream_port_t *amstream_find_port(const char *name);
void amstream_port_open(stream_port_t *this);
int amstream_port_release(stream_port_t *port);
int video_port_init(stream_port_t *port, struct stream_buf_s * pbuf);

void esparser_start_search(u32 parser_type, u32 phys_addr, u32 len);

void esparser_set_search_done_cb(void *data, void *cb);
void vh264_set_params_cb(void *data, void *cb);
void vp9_set_params_cb(void *data, void *cb);
bool vh264_output_is_starved(void);
int vh264_vififo_level(void);
bool vp9_output_is_starved(const char *receiver, unsigned int low_level);
unsigned int vp9_vififo_level(void);
