// Copyright (c) 2017 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

/*
 * SPI bus thread
 */
void spiThread2(void* arg);

/*
 * CAN TX thread
 */
void canTxThreadFunc(void* arg);

/*
 * CAN RX thread
 */
void canRxThreadFunc(void* arg);

/**
 * @desc Performs periodic tasks every second
 */
void heartbeatThreadFunc(void* arg);

// @brief Convert bytes in RX buffer from ADC reading to temperature
// @return temp Temperature in degrees C, resolution of .25
uint8_t adcToTemp(uint8_t* rxbuf);
