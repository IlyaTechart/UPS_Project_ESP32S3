#pragma once

#include "esp_err.h"
#include "frames_structure.h"
#include "wifi_control.h"

#define SIZE_OF_CIRCULAR_BUFFER  300  //Колличесвто структур-кадров которые будут храниться в кольцевом буфере

typedef enum{
    RINGBUF_OK,
    RINGBUF_ERR,
    RINGBUF_PARAM_ERR,
    RINGBUF_OVERFLOW,
    RINGBUF_NULL_POINTER,
    RINGBUF_MUTEX_NOT_GIVE,
}RingBuffStatus_t;           // Должен отражать состояние кольцевого буфера 

typedef struct{
    ModulData_t *buffer;                // Буфер кадров
    volatile size_t tail;               // Точка чтения 
    volatile size_t head;               // Точка записи 
    volatile size_t size;               // Размер буфера 
    volatile size_t cell_size;          // Размер одной ячейки (байт)
    volatile bool is_full;              // Флаг переполнения
}RingBuffModulData_t;

void logger_Inint(void);

RingBuffStatus_t RingBuffWrite(ModulData_t* ModulData);

// Глобальная структура с RMS-значениями по данным от FPGA
extern FpgaRmsData_t gFpgaRmsData;