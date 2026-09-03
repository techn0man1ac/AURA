#include "hera_interface.h"
#include "stub_utils.h"
#include "images_data.h"

#define NULL ((void*)0)

#define UART_BASE 0x80000100
#define UART_DATA   (*(volatile unsigned char *)(UART_BASE + 0x00))
#define UART_STATUS (*(volatile unsigned int  *)(UART_BASE + 0x04))

void put_char(char c) {
    while ((UART_STATUS & 0x04) == 0);
    UART_DATA = c;
    if (c == '\n') {
        while ((UART_STATUS & 0x04) == 0);
        UART_DATA = '\r';
    }
}

void print_str(const char *s) {
    while (*s) {
        put_char(*s++);
    }
}

static void stub_memcpy(void* dest, const void* src, uint32 n) {
    uint8* d = (uint8*)dest;
    const uint8* s = (const uint8*)src;
    
    while (n--) {
        *d++ = *s++;
    }
}

void print_dec(uint32 val) {
    char buffer[12];
    int i = 0;
    if (val == 0) {
        print_str("0");
        return;
    }
    while (val > 0) {
        buffer[i++] = (val % 10) + '0';
        val /= 10;
    }
    while (i > 0) {
        put_char(buffer[--i]);
    }
}

void print_hex(uint32 val) {
    char hex_digits[] = "0123456789ABCDEF";
    print_str("0x");
    for (int i = 28; i >= 0; i -= 4) {
        put_char(hex_digits[(val >> i) & 0xF]);
    }
}

static void stub_print_data(void* pData, uint16 size) {
    uint8* pBytes = (uint8*)pData;
    print_str("[STUB-DATA] Content: ");
    for (uint16 i = 0; i < size; i++) {
        print_hex(pBytes[i]);
        print_str(" ");
    }
    print_str("\n");
}

#define LOOPS_PER_SEC 500000000UL 
static void simple_sleep(uint32 seconds) {
    volatile uint32 count = seconds * LOOPS_PER_SEC;
    while (count > 0) {
        count--;
        __asm__ volatile ("nop");
    }
}

static ControlBlock_t ctrl_block = { 
    .afc_state = AFC_IDLE,
    .hs_state = HS_IDLE,
    .tiri_state = TIRI_IDLE,
    .sleep_counter_sec = 0
};
static volatile ControlBlock_t* const pCtrl = &ctrl_block;

error_code_t Hera_AFC_StoreImage(void) {
    print_str("[STUB] Hera_AFC_StoreImage called.\n");
    if (pCtrl->afc_state != AFC_IMAGE_REQUEST_COMPLETED) return HERA_ERR_BUSY;

    pCtrl->afc_state = AFC_IMAGE_COPY_REQUESTED;
    print_str("[STUB] State: COPY_REQUESTED. Sleeping 3s...\n");
    simple_sleep(3);
    pCtrl->afc_state = AFC_IMAGE_COPY_COMPLETED;
    print_str("[STUB] Core0 finished. State: COPY_COMPLETED.\n");

    if (pCtrl->afc_state == AFC_IMAGE_COPY_COMPLETED) {
        pCtrl->afc_state = AFC_IDLE; 
        return HERA_OK;
    }
    return HERA_ERR_FAILURE;
}

static uint8_t current_image_idx = 0; 

error_code_t Hera_AFC_AcquireSingleImage(uint32 exp_us) {
    print_str("[STUB] Hera_AFC_AcquireSingleImage. Exp: ");
    print_dec(exp_us);
    print_str(" ms.\n");

    if (pCtrl->afc_state != AFC_IDLE) {
        print_str("[STUB] Error: AFC Busy.\n");
        return HERA_ERR_BUSY;
    }

    pCtrl->afc_exposure_time_us = exp_us;
    pCtrl->afc_state = AFC_IMAGE_REQUESTED;
    
    print_str("[STUB] Simulating HW: Loading Image Index: ");
    print_dec(current_image_idx + 1);
    print_str(" / ");
    print_dec(NUM_AFC_IMAGES);
    print_str("\n");

    const uint8_t* src_image = afc_images_bank[current_image_idx];

    //copy to the shared area
    stub_memcpy((void*)pCtrl->afc_image_buffer, (const void*)src_image, AFC_IMG_SIZE);
    
    current_image_idx++;
    if (current_image_idx >= NUM_AFC_IMAGES) {
        current_image_idx = 0; //loop back
    }

    print_str("[STUB] Sleeping 7s...\n");
    simple_sleep(7);

    pCtrl->afc_state = AFC_IMAGE_REQUEST_COMPLETED;
    print_str("[STUB] AFC Image Request Completed.\n");

    pCtrl->afc_state = AFC_IDLE;
    print_str("[STUB] Woke up.\n");

    return HERA_OK;}

uint8* Hera_AFC_GetImageBuffer(void) {
    return (uint8*)pCtrl->afc_image_buffer;
}

error_code_t Hera_HS_AcquireImage(uint32 exp_ms) {
    print_str("[STUB] Hera_HS_AcquireImage called.\n");
    if (pCtrl->hs_state != HS_IDLE) return HERA_ERR_BUSY;
    pCtrl->hs_state = HS_ACQUIRE_COPY_REQUESTED;
    simple_sleep(900); // 15 minutes
    pCtrl->hs_state = HS_ACQUIRE_COPY_COMPLETED;
    print_str("[STUB] HS Done.\n");
    pCtrl->hs_state = HS_IDLE; 
    return HERA_OK;
}

error_code_t Hera_TIRI_AcquireImage(void) {
    print_str("[STUB] Hera_TIRI_AcquireImage called.\n");
    if (pCtrl->tiri_state != TIRI_IDLE) return HERA_ERR_BUSY;
    pCtrl->tiri_state = TIRI_ACQUIRE_COPY_REQUESTED;
    simple_sleep(600); // 10 minutes
    pCtrl->tiri_state = TIRI_ACQUIRE_COPY_COMPLETED;
    print_str("[STUB] TIRI Done.\n");
    pCtrl->tiri_state = TIRI_IDLE;
    return HERA_OK;
}

error_code_t Hera_Sleep(uint32 seconds) {
    print_str("[STUB] Sleep: "); print_dec(seconds); print_str("s.\n");
    pCtrl->sleep_counter_sec = seconds;
    pCtrl->sleep_active_flag = 1;
    simple_sleep(seconds);
    pCtrl->sleep_counter_sec = 0;
    pCtrl->sleep_active_flag = 0;
    return HERA_OK;
}

error_code_t Hera_HK_Report(uint16 sid, void* pData, uint16 size) {
    print_str("[STUB] HK SID: "); print_dec(sid); print_str("\n");
    if (pData) stub_print_data(pData, size);
    simple_sleep(1);
    return HERA_OK;
}

error_code_t Hera_Event_Report(uint16 event_id, void* pData, uint16 size) {
    print_str("[STUB] EVENT ID: "); print_dec(event_id); print_str("\n");
    if (pData) stub_print_data(pData, size);
    simple_sleep(1);
    return HERA_OK;
}

error_code_t Hera_Science_Report(uint16 apid, uint8 type, uint8 subtype, void* pData, uint16 size) {
    print_str("[STUB] SCIENCE APID: "); print_dec(apid); print_str("\n");
    print_str("[STUB] SCIENCE Type: "); print_dec(type); print_str("\n");
    print_str("[STUB] SCIENCE Subtype: "); print_dec(subtype); print_str("\n");
    print_str("[STUB] SCIENCE Size: "); print_dec(size); print_str("\n");
    print_str("\n");
    simple_sleep(1);
    return HERA_OK;
}
