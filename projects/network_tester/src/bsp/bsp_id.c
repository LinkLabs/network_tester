#include "em_system.h"
#include "em_msc.h"
#include "bsp_id.h"

#include "debug_print.h"

#define USER_PAGE_EUI64_ADDR_LO    (USERDATA_BASE      )
#define USER_PAGE_EUI64_ADDR_HI    (USERDATA_BASE + 0x4)
#define USER_PAGE_EUI64_ADDR_CHK   (USERDATA_BASE + 0x8)

static uint32_t compute_eui64_check_word(uint64_t in)
{
    uint32_t n;
    uint8_t chk = 0;
    uint8_t* p = (uint8_t*)&in;

    for(n=0; n<sizeof(uint64_t); n++)
    {
        chk += p[n];
    }
    return(chk);
}


static uint64_t get_stored_eui64(void)
{
    uint32_t id_lo = *(uint32_t*)USER_PAGE_EUI64_ADDR_LO;
    uint32_t id_hi = *(uint32_t*)USER_PAGE_EUI64_ADDR_HI;
    uint64_t id = (((uint64_t)id_hi) << 32) | ((uint64_t)id_lo);

    uint8_t id_chk = (uint8_t)(*(uint32_t*)USER_PAGE_EUI64_ADDR_CHK);

    if ( compute_eui64_check_word(id) == id_chk)
    {
        return(id);
    }
    else
    {
        return(0);
    }
}

uint64_t bsp_get_unique(void)
{
    uint64_t stored_eui64 = get_stored_eui64();
    if (stored_eui64 != 0)
    {
        return (stored_eui64);
    }
    else
    {
        Debug_Printf("EUI-64 Not found in USER PAGE! Using CPU ID instead.\n");
        return (SYSTEM_GetUnique());
    }
}
