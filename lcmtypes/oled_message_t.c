// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "oled_message_t.h"

static int __oled_message_t_hash_computed;
static uint64_t __oled_message_t_hash;

uint64_t __oled_message_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __oled_message_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__oled_message_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0xfcdfd38929ceee80LL
         + __int64_t_hash_recursive(&cp)
         + __string_hash_recursive(&cp)
         + __string_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __oled_message_t_get_hash(void)
{
    if (!__oled_message_t_hash_computed) {
        __oled_message_t_hash = (int64_t)__oled_message_t_hash_recursive(NULL);
        __oled_message_t_hash_computed = 1;
    }

    return __oled_message_t_hash;
}

int __oled_message_t_encode_array(void *buf, int offset, int maxlen, const oled_message_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __string_encode_array(buf, offset + pos, maxlen - pos, &(p[element].line1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __string_encode_array(buf, offset + pos, maxlen - pos, &(p[element].line2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int oled_message_t_encode(void *buf, int offset, int maxlen, const oled_message_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __oled_message_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __oled_message_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __oled_message_t_encoded_array_size(const oled_message_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

        size += __string_encoded_array_size(&(p[element].line1), 1);

        size += __string_encoded_array_size(&(p[element].line2), 1);

    }
    return size;
}

int oled_message_t_encoded_size(const oled_message_t *p)
{
    return 8 + __oled_message_t_encoded_array_size(p, 1);
}

size_t oled_message_t_struct_size(void)
{
    return sizeof(oled_message_t);
}

int oled_message_t_num_fields(void)
{
    return 3;
}

int oled_message_t_get_field(const oled_message_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= oled_message_t_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            f->name = "utime";
            f->type = LCM_FIELD_INT64_T;
            f->typestr = "int64_t";
            f->num_dim = 0;
            f->data = (void *) &p->utime;
            return 0;
        }
        
        case 1: {
            f->name = "line1";
            f->type = LCM_FIELD_STRING;
            f->typestr = "string";
            f->num_dim = 0;
            f->data = (void *) &p->line1;
            return 0;
        }
        
        case 2: {
            f->name = "line2";
            f->type = LCM_FIELD_STRING;
            f->typestr = "string";
            f->num_dim = 0;
            f->data = (void *) &p->line2;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *oled_message_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) oled_message_t_encode;
        typeinfo.decode         = (lcm_decode_t) oled_message_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) oled_message_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) oled_message_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  oled_message_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) oled_message_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) oled_message_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __oled_message_t_get_hash;
    }
    
    return &typeinfo;
}
int __oled_message_t_decode_array(const void *buf, int offset, int maxlen, oled_message_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __string_decode_array(buf, offset + pos, maxlen - pos, &(p[element].line1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __string_decode_array(buf, offset + pos, maxlen - pos, &(p[element].line2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __oled_message_t_decode_array_cleanup(oled_message_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

        __string_decode_array_cleanup(&(p[element].line1), 1);

        __string_decode_array_cleanup(&(p[element].line2), 1);

    }
    return 0;
}

int oled_message_t_decode(const void *buf, int offset, int maxlen, oled_message_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __oled_message_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __oled_message_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int oled_message_t_decode_cleanup(oled_message_t *p)
{
    return __oled_message_t_decode_array_cleanup(p, 1);
}

int __oled_message_t_clone_array(const oled_message_t *p, oled_message_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

        __string_clone_array(&(p[element].line1), &(q[element].line1), 1);

        __string_clone_array(&(p[element].line2), &(q[element].line2), 1);

    }
    return 0;
}

oled_message_t *oled_message_t_copy(const oled_message_t *p)
{
    oled_message_t *q = (oled_message_t*) malloc(sizeof(oled_message_t));
    __oled_message_t_clone_array(p, q, 1);
    return q;
}

void oled_message_t_destroy(oled_message_t *p)
{
    __oled_message_t_decode_array_cleanup(p, 1);
    free(p);
}

int oled_message_t_publish(lcm_t *lc, const char *channel, const oled_message_t *p)
{
      int max_data_size = oled_message_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = oled_message_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _oled_message_t_subscription_t {
    oled_message_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void oled_message_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    oled_message_t p;
    memset(&p, 0, sizeof(oled_message_t));
    status = oled_message_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding oled_message_t!!!\n", status);
        return;
    }

    oled_message_t_subscription_t *h = (oled_message_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    oled_message_t_decode_cleanup (&p);
}

oled_message_t_subscription_t* oled_message_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    oled_message_t_handler_t f, void *userdata)
{
    oled_message_t_subscription_t *n = (oled_message_t_subscription_t*)
                       malloc(sizeof(oled_message_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 oled_message_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg oled_message_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int oled_message_t_subscription_set_queue_capacity (oled_message_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int oled_message_t_unsubscribe(lcm_t *lcm, oled_message_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe oled_message_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

