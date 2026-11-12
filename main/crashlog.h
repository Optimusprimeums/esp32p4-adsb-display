#pragma once

/* call early in app_main: picks up the record of the last panic, if any */
void crashlog_init(void);
/* "" or a one-line description of why the previous run ended (panic, watchdog...) */
const char *crashlog_text(void);
