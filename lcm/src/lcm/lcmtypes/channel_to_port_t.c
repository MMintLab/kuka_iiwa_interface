/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <string.h>
#include <lcm/lcmtypes/channel_to_port_t.h>

static int __channel_to_port_t_hash_computed;
static int64_t __channel_to_port_t_hash;

int64_t __channel_to_port_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __channel_to_port_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__channel_to_port_t_get_hash;
    (void) cp;

    int64_t hash = 0x11dde9fa42a43913LL
         + __string_hash_recursive(&cp)
         + __int16_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __channel_to_port_t_get_hash(void)
{
    if (!__channel_to_port_t_hash_computed) {
        __channel_to_port_t_hash = __channel_to_port_t_hash_recursive(NULL);
        __channel_to_port_t_hash_computed = 1;
    }

    return __channel_to_port_t_hash;
}

int __channel_to_port_t_encode_array(void *buf, int offset, int maxlen, const channel_to_port_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __string_encode_array(buf, offset + pos, maxlen - pos, &(p[element].channel), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].port), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int channel_to_port_t_encode(void *buf, int offset, int maxlen, const channel_to_port_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __channel_to_port_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __channel_to_port_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __channel_to_port_t_encoded_array_size(const channel_to_port_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __string_encoded_array_size(&(p[element].channel), 1);

        size += __int16_t_encoded_array_size(&(p[element].port), 1);

    }
    return size;
}

int channel_to_port_t_encoded_size(const channel_to_port_t *p)
{
    return 8 + __channel_to_port_t_encoded_array_size(p, 1);
}

int __channel_to_port_t_decode_array(const void *buf, int offset, int maxlen, channel_to_port_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __string_decode_array(buf, offset + pos, maxlen - pos, &(p[element].channel), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].port), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __channel_to_port_t_decode_array_cleanup(channel_to_port_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __string_decode_array_cleanup(&(p[element].channel), 1);

        __int16_t_decode_array_cleanup(&(p[element].port), 1);

    }
    return 0;
}

int channel_to_port_t_decode(const void *buf, int offset, int maxlen, channel_to_port_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __channel_to_port_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __channel_to_port_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int channel_to_port_t_decode_cleanup(channel_to_port_t *p)
{
    return __channel_to_port_t_decode_array_cleanup(p, 1);
}

int __channel_to_port_t_clone_array(const channel_to_port_t *p, channel_to_port_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __string_clone_array(&(p[element].channel), &(q[element].channel), 1);

        __int16_t_clone_array(&(p[element].port), &(q[element].port), 1);

    }
    return 0;
}

channel_to_port_t *channel_to_port_t_copy(const channel_to_port_t *p)
{
    channel_to_port_t *q = (channel_to_port_t*) malloc(sizeof(channel_to_port_t));
    __channel_to_port_t_clone_array(p, q, 1);
    return q;
}

void channel_to_port_t_destroy(channel_to_port_t *p)
{
    __channel_to_port_t_decode_array_cleanup(p, 1);
    free(p);
}

