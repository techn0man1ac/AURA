#define UART_BASE 0x80000100
#define UART_RBR_THR ((volatile unsigned char *)(UART_BASE + 0))
#define UART_LSR     ((volatile unsigned char *)(UART_BASE + 5))
#define LSR_THRE 0x20

#define IMAGE_ADDRESS 0x40600000
#define IMG_WIDTH     1020
#define IMG_HEIGHT    1020

#define THRESHOLD     45    /* Поріг локального контрасту для виявлення орієнтира */
#define MAX_LANDMARKS 1024    /* Жорсткий ліміт точок на кадр для гарантії WCET */

void uart_putc(char c) {
    while (!(*UART_LSR & LSR_THRE));
    *UART_RBR_THR = c;
}

void uart_print(const char *str) {
    while (*str) {
        uart_putc(*str);
        str++;
    }
}

void uart_send_uint32(unsigned int packet) {
    for (int i = 3; i >= 0; i--) {
        while (!(*UART_LSR & LSR_THRE));
        *UART_RBR_THR = (unsigned char)((packet >> (i * 8)) & 0xFF);
    }
}

/* Новий анізотропний пайплайн виділення хмари точок (Замість блокової ентропії) */
void process_landmarkCloud(void) {
    unsigned char *grayscale_pixels = (unsigned char *)IMAGE_ADDRESS;
    unsigned int packed_packet = 0;
    unsigned int landmarks_found = 0;

    /* Скануємо кадр кроком 12 пікселів, відступаючи від країв матриці камери */
    for (int y = 15; y < IMG_HEIGHT - 15; y += 12) {
        for (int x = 15; x < IMG_WIDTH - 15; x += 12) {
            
            /* Детерміністичний вихід при досягненні ліміту пропускної здатності */
            if (landmarks_found >= MAX_LANDMARKS) {
                break;
            }

            /* Розрахунок локальних градієнтів за допомогою фіксованого хреста зміщення */
            unsigned char p_top   = grayscale_pixels[(y - 8) * IMG_WIDTH + x];
            unsigned char p_down  = grayscale_pixels[(y + 8) * IMG_WIDTH + x];
            unsigned char p_left  = grayscale_pixels[y * IMG_WIDTH + (x - 8)];
            unsigned char p_right = grayscale_pixels[y * IMG_WIDTH + (x + 8)];

            /* Обчислення модулів абсолютних різниць на цілих числах (Без FPU) */
            int diff_v = (int)p_top - (int)p_down;
            int diff_h = (int)p_left - (int)p_right;
            
            if (diff_v < 0) diff_v = -diff_v;
            if (diff_h < 0) diff_h = -diff_h;

            int total_score = diff_v + diff_h;

            /* Якщо локальний контраст перевищує поріг (знайдено кут скелі / кратер) */
            if (total_score > THRESHOLD) {
                /* Масштабуємо якість до 4 біт (0..15) */
                unsigned int packed_score = (unsigned int)(total_score >> 4);
                if (packed_score > 15) packed_score = 15;

                /* УЛЬТРАКОМПАКТНЕ БІТОВЕ ПАКУВАННЯ ПІД НОВИЙ PYTHON ДЕКОДЕР (0xBD) */
                packed_packet = 0;
                packed_packet |= ((unsigned int)0xBD << 24);         /* [31:24] Маркер синхронізації */
                packed_packet |= ((unsigned int)(x & 0x03FF) << 14);  /* [23:14] Координата X (10 біт) */
                packed_packet |= ((unsigned int)(y & 0x03FF) << 4);   /* [13:4]  Координата Y (10 біт) */
                packed_packet |= (unsigned int)(packed_score & 0x0F); /* [3:0]   Якість / Контраст */

                /* Безпечне побайтове вимивання регістра в UART */
                uart_send_uint32(packed_packet);
                landmarks_found++;
            }
        }
        if (landmarks_found >= MAX_LANDMARKS) {
            break;
        }
    }

    /* Сигналізуємо Ground Segment про фінал обробки кадру */
    packed_packet = (0xFE << 24);
    uart_send_uint32(packed_packet);
}

/* ФІКС: Точка входу змінена на int main() відповідно до вимог BSP LEON3 */
int main(void) {
    for (volatile int i = 0; i < 100000; i++);

    uart_print("\n===================================\n");
    uart_print("AURA Firmware: Autonomous Landmark Cloud\n");
    uart_print("===================================\n\n");

    process_landmarkCloud();

    while (1);
    return 0;
}