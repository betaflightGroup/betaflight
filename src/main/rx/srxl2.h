#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "pg/rx.h"

struct rxRuntimeState_s;
struct sbuf_s;

bool srxl2RxInit(const rxConfig_t *rxConfig, struct rxRuntimeState_s *rxRuntimeState);
void srxl2RxWriteData(const void *data, int len);
bool srxl2TelemetryRequested(void);
void srxl2InitializeFrame(struct sbuf_s *dst);
void srxl2FinalizeFrame(struct sbuf_s *dst);
void srxl2Bind(void);
