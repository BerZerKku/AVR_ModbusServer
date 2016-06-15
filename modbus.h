/*
 * modbus.h
 *
 *  Created on: 09.06.2016
 *      Author: Shcheblykin
 */

#ifndef MODBUS_H_
#define MODBUS_H_

#include "define.h"

/// Состояния получения данных
typedef enum {
	MODBUS_STATE_RX_OK,			///< в сообщении ошибок нет
	MODBUS_STATE_RX_ERROR_LEN,	///< ошибочная длина полученного сообщения
	MODBUS_STATE_RX_ERROR_ADR,	///< ошибочный адрес устройства
	MODBUS_STATE_RX_ERROR_FUNC,	///< ошибочная функция
	MODBUS_STATE_RX_ERROR_NUM,	///< ошибочное количество принятых регистров
	MODBUS_STATE_RX_ERROR_CRC,	///< ошибочная КС
	MODBUS_STATE_RX_ERROR_DATA	///< ошибочные данные
} MODBUS_STATE_RX;

/// Коды функций MODBUS
typedef enum {
	MODBUS_FUNCTION_NO						= 0x00,	///< нет
	MODBUS_FUNCTION_READ_HOLDING_REGISTERS 	= 0x03,	///< считывание регистров
	MODBUS_FUNCTION_WRITE_SINGLE_REGISTER	= 0x06	///< запись регистра
} MODBUS_FUNCTION;

/**	Извлечение данных из принятого сообщения READ_HOLDING_REGISTERS - 0x03.
 *
 *	@param[in] *buf Указатель на буфер формируемого сообщения.
 *	@param[in] adrReg Адрес первого регистра для чтения.
 *	@param[out] Значение регистра.
 *	@return Значение параметра, если присут
 */
uint8_t MODBUS_getHoldingRegisters(uint8_t *buf, uint16_t adrReg, uint16_t* val);

/**	Чтение регистров - READ_HOLDING_REGISTERS - 0x03.
 *
 *	Адрес регистра при отправке уменьшается на 1.
 *
 *	@param[in] *buf Указатель на буфер формируемого сообщения.
 * 	@param[in] adrD Адрес устройства в сети MODBUS.
 * 	@param[in] adrReg Адрес первого регистра для чтения.
 * 	@param[in] numReg Количество регистров для чтения.
 * 	@return Количество байт данных на передачу.
 */
uint8_t MODBUS_readHoldingRegisters(uint8_t *buf, uint8_t adrDev,
		uint16_t adrReg, uint8_t numReg);

/**	Запись регистра - WRITE_SINGLE_REGISTER - 0х06.
 *
 *	Адрес регистра при отправке уменьшается на 1.
 *
 *	@param[in] *buf Указатель на буфер формируемого сообщения.
 * 	@param[in] adrD Адрес устройства в сети MODBUS.
 * 	@param[in] adrReg Адрес регистра для записи.
 * 	@param[in] val Значение регистра.
 * 	@return Количество байт данных на передачу.
 */
uint8_t MODBUS_writeSingleRegister(uint8_t *buf, uint8_t adrDev,
		uint16_t adrReg, uint16_t val);

/**	Проверка принятого сообщения.
 *
 *	@param[in] *bufRx Указатель на буфер принятого сообщения.
 *	@param[in] *bufTx Указатель на буфер переданного сообщения.
 *	@param[in] adrD Адрес устройства в сети MODBUS.
 *	@param[in] len Количество принятых байт данных.
 * 	@return
 */
MODBUS_STATE_RX MODBUS_checkReadData(uint8_t *bufRx, uint8_t *bufTx, uint8_t len);

#endif /* MODBUS_H_ */
