// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "robot_path_t.h"

static int __robot_path_t_hash_computed;
static uint64_t __robot_path_t_hash;

uint64_t __robot_path_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __robot_path_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__robot_path_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0xd8a57fd0b3392990LL
         + __int64_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __pose_xyt_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __robot_path_t_get_hash(void)
{
    if (!__robot_path_t_hash_computed) {
        __robot_path_t_hash = (int64_t)__robot_path_t_hash_recursive(NULL);
        __robot_path_t_hash_computed = 1;
    }

    return __robot_path_t_hash;
}

int __robot_path_t_encode_array(void *buf, int offset, int maxlen, const robot_path_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].path_length), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __pose_xyt_t_encode_array(buf, offset + pos, maxlen - pos, p[element].path, p[element].path_length);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int robot_path_t_encode(void *buf, int offset, int maxlen, const robot_path_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __robot_path_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __robot_path_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __robot_path_t_encoded_array_size(const robot_path_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

        size += __int32_t_encoded_array_size(&(p[element].path_length), 1);

        size += __pose_xyt_t_encoded_array_size(p[element].path, p[element].path_length);

    }
    return size;
}

int robot_path_t_encoded_size(const robot_path_t *p)
{
    return 8 + __robot_path_t_encoded_array_size(p, 1);
}

size_t robot_path_t_struct_size(void)
{
    return sizeof(robot_path_t);
}

int robot_path_t_num_fields(void)
{
    return 3;
}

int robot_path_t_get_field(const robot_path_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= robot_path_t_num_fields())
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
            f->name = "path_length";
            f->type = LCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void *) &p->path_length;
            return 0;
        }
        
        case 2: {
            /* pose_xyt_t */
            f->name = "path";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "pose_xyt_t";
            f->num_dim = 1;
            f->dim_size[0] = p->path_length;
            f->dim_is_variable[0] = 1;
            f->data = (void *) &p->path;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *robot_path_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) robot_path_t_encode;
        typeinfo.decode         = (lcm_decode_t) robot_path_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) robot_path_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) robot_path_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  robot_path_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) robot_path_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) robot_path_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __robot_path_t_get_hash;
    }
    
    return &typeinfo;
}
int __robot_path_t_decode_array(const void *buf, int offset, int maxlen, robot_path_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].path_length), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].path = (pose_xyt_t*) lcm_malloc(sizeof(pose_xyt_t) * p[element].path_length);
        thislen = __pose_xyt_t_decode_array(buf, offset + pos, maxlen - pos, p[element].path, p[element].path_length);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __robot_path_t_decode_array_cleanup(robot_path_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

        __int32_t_decode_array_cleanup(&(p[element].path_length), 1);

        __pose_xyt_t_decode_array_cleanup(p[element].path, p[element].path_length);
        if (p[element].path) free(p[element].path);

    }
    return 0;
}

int robot_path_t_decode(const void *buf, int offset, int maxlen, robot_path_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __robot_path_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __robot_path_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int robot_path_t_decode_cleanup(robot_path_t *p)
{
    return __robot_path_t_decode_array_cleanup(p, 1);
}

int __robot_path_t_clone_array(const robot_path_t *p, robot_path_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

        __int32_t_clone_array(&(p[element].path_length), &(q[element].path_length), 1);

        q[element].path = (pose_xyt_t*) lcm_malloc(sizeof(pose_xyt_t) * q[element].path_length);
        __pose_xyt_t_clone_array(p[element].path, q[element].path, p[element].path_length);

    }
    return 0;
}

robot_path_t *robot_path_t_copy(const robot_path_t *p)
{
    robot_path_t *q = (robot_path_t*) malloc(sizeof(robot_path_t));
    __robot_path_t_clone_array(p, q, 1);
    return q;
}

void robot_path_t_destroy(robot_path_t *p)
{
    __robot_path_t_decode_array_cleanup(p, 1);
    free(p);
}

int robot_path_t_publish(lcm_t *lc, const char *channel, const robot_path_t *p)
{
      int max_data_size = robot_path_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = robot_path_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _robot_path_t_subscription_t {
    robot_path_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void robot_path_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    robot_path_t p;
    memset(&p, 0, sizeof(robot_path_t));
    status = robot_path_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding robot_path_t!!!\n", status);
        return;
    }

    robot_path_t_subscription_t *h = (robot_path_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    robot_path_t_decode_cleanup (&p);
}

robot_path_t_subscription_t* robot_path_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    robot_path_t_handler_t f, void *userdata)
{
    robot_path_t_subscription_t *n = (robot_path_t_subscription_t*)
                       malloc(sizeof(robot_path_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 robot_path_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg robot_path_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int robot_path_t_subscription_set_queue_capacity (robot_path_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int robot_path_t_unsubscribe(lcm_t *lcm, robot_path_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe robot_path_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

