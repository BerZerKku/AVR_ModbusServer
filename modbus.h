/*
 * modbus.h
 *
 *  Created on: 09.06.2016
 *      Author: Shcheblykin
 */

#ifndef MODBUS_H_
#define MODBUS_H_

#include "define.h"

/// ��������� ��������� ������
typedef enum {
	MODBUS_STATE_RX_OK,			///< � ��������� ������ ���
	MODBUS_STATE_RX_ERROR_LEN,	///< ��������� ����� ����������� ���������
	MODBUS_STATE_RX_ERROR_ADR,	///< ��������� ����� ����������
	MODBUS_STATE_RX_ERROR_FUNC,	///< ��������� �������
	MODBUS_STATE_RX_ERROR_NUM,	///< ��������� ���������� �������� ���������
	MODBUS_STATE_RX_ERROR_CRC,	///< ��������� ��
	MODBUS_STATE_RX_ERROR_DATA	///< ��������� ������
} MODBUS_STATE_RX;

/// ���� ������� MODBUS
typedef enum {
	MODBUS_FUNCTION_NO						= 0x00,	///< ���
	MODBUS_FUNCTION_READ_HOLDING_REGISTERS 	= 0x03,	///< ���������� ���������
	MODBUS_FUNCTION_WRITE_SINGLE_REGISTER	= 0x06	///< ������ ��������
} MODBUS_FUNCTION;

/**	���������� ������ �� ��������� ��������� READ_HOLDING_REGISTERS - 0x03.
 *
 *	@param[in] *buf ��������� �� ����� ������������ ���������.
 *	@param[in] adrReg ����� ������� �������� ��� ������.
 *	@param[out] �������� ��������.
 *	@return �������� ���������, ���� ������
 */
uint8_t MODBUS_getHoldingRegisters(uint8_t *buf, uint16_t adrReg, uint16_t* val);

/**	������ ��������� - READ_HOLDING_REGISTERS - 0x03.
 *
 *	����� �������� ��� �������� ����������� �� 1.
 *
 *	@param[in] *buf ��������� �� ����� ������������ ���������.
 * 	@param[in] adrD ����� ���������� � ���� MODBUS.
 * 	@param[in] adrReg ����� ������� �������� ��� ������.
 * 	@param[in] numReg ���������� ��������� ��� ������.
 * 	@return ���������� ���� ������ �� ��������.
 */
uint8_t MODBUS_readHoldingRegisters(uint8_t *buf, uint8_t adrDev,
		uint16_t adrReg, uint8_t numReg);

/**	������ �������� - WRITE_SINGLE_REGISTER - 0�06.
 *
 *	����� �������� ��� �������� ����������� �� 1.
 *
 *	@param[in] *buf ��������� �� ����� ������������ ���������.
 * 	@param[in] adrD ����� ���������� � ���� MODBUS.
 * 	@param[in] adrReg ����� �������� ��� ������.
 * 	@param[in] val �������� ��������.
 * 	@return ���������� ���� ������ �� ��������.
 */
uint8_t MODBUS_writeSingleRegister(uint8_t *buf, uint8_t adrDev,
		uint16_t adrReg, uint16_t val);

/**	�������� ��������� ���������.
 *
 *	@param[in] *bufRx ��������� �� ����� ��������� ���������.
 *	@param[in] *bufTx ��������� �� ����� ����������� ���������.
 *	@param[in] adrD ����� ���������� � ���� MODBUS.
 *	@param[in] len ���������� �������� ���� ������.
 * 	@return
 */
MODBUS_STATE_RX MODBUS_checkReadData(uint8_t *bufRx, uint8_t *bufTx, uint8_t len);

#endif /* MODBUS_H_ */
