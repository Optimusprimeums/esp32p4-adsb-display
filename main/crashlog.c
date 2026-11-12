/* Remembers why the last panic happened, across the reboot, so it can be read
 * over the network (GET /status) when no serial console is attached.
 * esp_panic_handler is wrapped at link time (see CMakeLists.txt). */
#include "crashlog.h"

#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_private/panic_internal.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "riscv/rvruntime-frames.h"

#define MAGIC 0xC4A5410Du

typedef struct {
    uint32_t magic;
    uint32_t pc, ra, mcause, mtval;
    char reason[48];
    char task[16];
    char details[96];
} crash_t;

static RTC_NOINIT_ATTR crash_t s_crash;
static char s_text[256];

extern bool g_panic_abort;
extern char *g_panic_abort_details;
void __real_esp_panic_handler(panic_info_t *info);

void __wrap_esp_panic_handler(panic_info_t *info)
{
    memset(&s_crash, 0, sizeof(s_crash));
    const RvExcFrame *f = info->frame;
    if (f) {
        s_crash.pc = f->mepc;
        s_crash.ra = f->ra;
        s_crash.mcause = f->mcause;
        s_crash.mtval = f->mtval;
    }
    if (info->reason) strlcpy(s_crash.reason, info->reason, sizeof(s_crash.reason));
    if (g_panic_abort && g_panic_abort_details) strlcpy(s_crash.details, g_panic_abort_details, sizeof(s_crash.details));
    const char *t = pcTaskGetName(NULL);
    if (t) strlcpy(s_crash.task, t, sizeof(s_crash.task));
    s_crash.magic = MAGIC;
    __real_esp_panic_handler(info);
}

void crashlog_init(void)
{
    esp_reset_reason_t r = esp_reset_reason();
    if (r == ESP_RST_PANIC && s_crash.magic == MAGIC) {
        /* JSON-safe copy of the details */
        char d[96];
        size_t o = 0;
        for (const char *c = s_crash.details; *c && o < sizeof(d) - 1; c++) {
            if (*c != '"' && *c != '\\' && (unsigned char)*c >= 0x20) d[o++] = *c;
        }
        d[o] = 0;
        snprintf(s_text, sizeof(s_text), "%s in %s pc=0x%08lx ra=0x%08lx mcause=%lu mtval=0x%08lx %s", s_crash.reason,
                 s_crash.task, (unsigned long)s_crash.pc, (unsigned long)s_crash.ra, (unsigned long)s_crash.mcause,
                 (unsigned long)s_crash.mtval, d);
    } else if (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT) {
        snprintf(s_text, sizeof(s_text), "watchdog reset (%d)", (int)r);
    } else if (r == ESP_RST_BROWNOUT) {
        strlcpy(s_text, "brownout", sizeof(s_text));
    }
    s_crash.magic = 0;
}

const char *crashlog_text(void)
{
    return s_text;
}
