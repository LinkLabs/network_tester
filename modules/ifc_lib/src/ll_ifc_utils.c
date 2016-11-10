#include "ll_ifc_utils.h"
#include "ll_ifc.h" //for gettime()

uint16_t crc16(char *ptr, int count)
{
    int  crc;
    char i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (int) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return (crc);
}

uint32_t crc32(uint32_t crc, uint8_t *buf, size_t len)
{
    int k;

    crc = ~crc;
    while (len--)
    {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
        {
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
        }
    }
    return ~crc;
}

int32_t ll_difftime(struct time *a, struct time *b)
{
    long sec;
    long nsec;

    if (a == NULL || b == NULL)
    {
        return 0;
    }

    sec = b->tv_sec - a->tv_sec;
    nsec = b->tv_nsec - a->tv_nsec;
    if (nsec < 0)
    {
        sec--;
    }

    return sec;
}

int32_t ll_difftime_from_now(struct time *a)
{
    struct time b;
    gettime(&b);
    return ll_difftime(a, &b);
}
