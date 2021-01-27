#include <string.h>
#include "ModbusSlave.h"

/**
 * ---------------------------------------------------
 *                CONSTANTS AND MACROS
 * ---------------------------------------------------
 */

#define MODBUS_FRAME_SIZE 4
#define MODBUS_CRC_LENGTH 2

#define MODBUS_ADDRESS_INDEX 0
#define MODBUS_FUNCTION_CODE_INDEX 1
#define MODBUS_DATA_INDEX 2

#define MODBUS_BROADCAST_ADDRESS 00
#define MODBUS_ADDRESS_MIN 1
#define MODBUS_ADDRESS_MAX 247

#define MODBUS_HALF_SILENCE_MULTIPLIER 3
#define MODBUS_FULL_SILENCE_MULTIPLIER 7

#define readUInt16(arr, index) word(arr[index], arr[index + 1])
#define readCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) + 1], arr[length - MODBUS_CRC_LENGTH])

/**
 * ---------------------------------------------------
 *                  PUBLIC METHODS
 * ---------------------------------------------------
 */

/**
 * Inicializar un objeto esclavo Modbus.
 *
 * @param unitAddress La dirección de la unidad esclavo modbus.
 */
ModbusSlave::ModbusSlave(uint8_t unitAddress)
{
    ModbusSlave::setUnitAddress(unitAddress);
}

/**
 * Obtener la dirección de la unidad esclava modbus.
 */
uint8_t ModbusSlave::getUnitAddress()
{
    return _unitAddress;
}

/**
 * Establece la dirección de la unidad esclavo modbus.
 *
 * @param unitAddress La dirección de la unidad de esclavos modbus.
 */
void ModbusSlave::setUnitAddress(uint8_t unitAddress)
{
    if (unitAddress < MODBUS_ADDRESS_MIN || unitAddress > MODBUS_ADDRESS_MAX)
    {
        return;
    }
    _unitAddress = unitAddress;
}

/**
 * Inicializa el objeto modbus.
 *
 * @param unitAddress La dirección de la unidad esclavo modbus.
 * @param TransmissionControlPin El pin de salida digital que se utilizará para el control de transmisión RS485.
 */
Modbus::Modbus(uint8_t unitAddress, int transmissionControlPin)
    : Modbus(Serial, unitAddress, transmissionControlPin)
{
}

/**
 * Inicializa el objeto modbus.
 *
 * @param serialStream El flujo serial usado para la comunicación Modbus.
 * @param unitAddress La dirección de la unidad esclavo modbus.
 * @param TransmissionControlPin El pin de salida digital que se utilizará para el control de transmisión RS485.
 */
Modbus::Modbus(Stream &serialStream, uint8_t unitAddress, int transmissionControlPin)
    : _serialStream(serialStream)
{
    // Establece el ID de la unidad esclavo modbus.
    _slaves[0].setUnitAddress(unitAddress);
    cbVector = _slaves[0].cbVector;

    // Establecer el pin de control de transmisión para la comunicación RS485.
    _transmissionControlPin = transmissionControlPin;
}

/**
 * Inicializa el objeto modbus.
 *
 * @param slaves Puntero a una matriz de ModbusSlaves.
 * @param numberOfSlaves El número de ModbusSlaves en la matriz.
 * @param TransmissionControlPin El pin de salida digital que se utilizará para el control de transmisión RS485.
 */
Modbus::Modbus(ModbusSlave *slaves, uint8_t numberOfSlaves, int transmissionControlPin)
    : Modbus(Serial, slaves, numberOfSlaves, transmissionControlPin)
{
}

/**
 * Inicializa el objeto modbus.
 *
 * @param serialStream el flujo serial usado para la comunicación modbus.
 * @param slaves Puntero a una matriz de ModbusSlaves.
 * @param numberOfSlaves El número de ModbusSlaves en la matriz.
 * @param TransmissionControlPin El pin de salida digital que se utilizará para el control de transmisión RS485.
 */
Modbus::Modbus(Stream &serialStream, ModbusSlave *slaves, uint8_t numberOfSlaves, int transmissionControlPin)
    : _serialStream(serialStream)
{
    // Establecer los esclavos modbus.
    _slaves = slaves;
    _numberOfSlaves = numberOfSlaves;
    cbVector = _slaves[0].cbVector;

    // Establecer el pin de control de transmisión para la comunicación RS485.
    _transmissionControlPin = transmissionControlPin;
}

/**
 * Establece la dirección de la unidad esclavo modbus.
 *
 * @param unitAddress La dirección de la unidad de esclavos modbus.
 */
void Modbus::setUnitAddress(uint8_t unitAddress)
{
    _slaves[0].setUnitAddress(unitAddress);
}

/**
 * Obtiene el número total de bytes enviados.
 *
 * @return El número de bytes.
 */
uint64_t Modbus::getTotalBytesSent()
{
    return _totalBytesSent;
}

/**
 * Obtiene el número total de bytes recibidos.
 *
 * @return El número de bytes.
 */
uint64_t Modbus::getTotalBytesReceived()
{
    return _totalBytesReceived;
}

/**
 * Comienza a inicializar el flujo en serie y se prepara para leer los mensajes de solicitud.
 *
 * @param baudrate La velocidad en baudios del puerto serie.
 */
void Modbus::begin(uint64_t baudrate)
{
    // Inicialice el pin de control de transmisión y establezca su estado.
    if (_transmissionControlPin > MODBUS_CONTROL_PIN_NONE)
    {
        pinMode(_transmissionControlPin, OUTPUT);
        digitalWrite(_transmissionControlPin, LOW);
    }

    // Deshabilita el tiempo de espera de la secuencia serial y limpia el búfer.
    _serialStream.setTimeout(0);
    _serialStream.flush();
    _serialTransmissionBufferLength = _serialStream.availableForWrite();

    // Calcula el tiempo de medio carácter basado en la velocidad en baudios de la serie.
    if (baudrate > 19200)
    {
        _halfCharTimeInMicroSecond = 250; // 0.5T.
    }
    else
    {
        _halfCharTimeInMicroSecond = 5000000 / baudrate; // 0.5T.
    }

    // Establezca la última hora recibida en 3.5T en el futuro para ignorar la solicitud actualmente en medio de la transmisión.
    _lastCommunicationTime = micros() + (_halfCharTimeInMicroSecond * MODBUS_FULL_SILENCE_MULTIPLIER);

    // Establece la longitud del búfer de solicitud en cero.
    _requestBufferLength = 0;
}

/**
 * Devuelve el código de función del mensaje de solicitud actual.
 *
 * @return Un byte que contiene el código de función del mensaje de solicitud actual.
 */
uint8_t Modbus::readFunctionCode()
{
    if (_requestBufferLength >= MODBUS_FRAME_SIZE && !_isRequestBufferReading)
    {
        return _requestBuffer[MODBUS_FUNCTION_CODE_INDEX];
    }
    return FC_INVALID;
}

/**
 * Devuelve la dirección de la unidad de destino del mensaje de solicitud actual.
 *
 * @return Un byte que contiene la dirección de la unidad de mensaje de solicitud actual.
 */
uint8_t Modbus::readUnitAddress()
{
    if ((_requestBufferLength >= MODBUS_FRAME_SIZE) && !_isRequestBufferReading)
    {
        return _requestBuffer[MODBUS_ADDRESS_INDEX];
    }
    return MODBUS_INVALID_UNIT_ADDRESS;
}

/**
 * Lee un estado de bobina del búfer de entrada.
 *
 * @param offset El offset de la primera bobina en el búfer.
 * @return El estado de la bobina del búfer (verdadero / falso).
 */
bool Modbus::readCoilFromBuffer(int offset)
{
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_COIL)
    {
        if (offset == 0)
        {
            // (2 x coilAddress, 1 x value).
            return readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2) == COIL_ON;
        }
        return false;
    }
    else if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_MULTIPLE_COILS)
    {
        // (2 x firstCoilAddress, 2 x coilsCount, 1 x valueBytes, n x values).
        uint16_t index = MODBUS_DATA_INDEX + 5 + (offset / 8);
        uint8_t bitIndex = offset % 8;
  
        // Verifica el desplazamiento.
        if (index < _requestBufferLength - MODBUS_CRC_LENGTH)
        {
            return bitRead(_requestBuffer[index], bitIndex);
        }
    }
    return false;
}

/**
 * Lee un valor de registro del búfer de entrada.
 *
 * @param offset El offset desde el primer registro en el búfer.
 * @return El valor de registro del búfer.
 */
uint16_t Modbus::readRegisterFromBuffer(int offset)
{
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_REGISTER)
    {
        if (offset == 0)
        {
            // (2 x coilAddress, 2 x value).
            return readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);
        }
    }
    else if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_WRITE_MULTIPLE_REGISTERS)
    {
        // (2 x firstRegisterAddress, 2 x registersCount, 1 x valueBytes, n x values).
        uint16_t index = MODBUS_DATA_INDEX + 5 + (offset * 2);

        // Check the offset.
        if (index < _requestBufferLength - MODBUS_CRC_LENGTH)
        {
            return readUInt16(_requestBuffer, index);
        }
    }
    return 0;
}

/**
 * Escribe el estado de la excepción en el búfer.
 *
 * @param desplazamiento
 * @param status Indicador de estado de excepción (true / false)
 */
uint8_t Modbus::writeExceptionStatusToBuffer(int offset, bool status)
{
    // Verifique el código de la función.
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_EXCEPTION_STATUS)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // (1 x values).
    uint16_t index = MODBUS_DATA_INDEX;
    uint8_t bitIndex = offset % 8;

    // Verifica el desplazamiento.
    if (index >= _responseBufferLength - MODBUS_CRC_LENGTH)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    if (status)
    {
        bitSet(_responseBuffer[index], bitIndex);
    }
    else
    {
        bitClear(_responseBuffer[index], bitIndex);
    }

    return STATUS_OK;
}

/**
 * Escribe el estado de la bobina en el búfer de salida.
 *
 * @param offset El offset de la primera bobina en el búfer.
 * @param state El estado para escribir en el búfer (verdadero / falso).
 */
uint8_t Modbus::writeCoilToBuffer(int offset, bool state)
{  
    // Verifique el código de la función.
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_DISCRETE_INPUT &&
        _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_COILS)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // (1 x valueBytes, n x values).
    uint16_t index = MODBUS_DATA_INDEX + 1 + (offset / 8);
    uint8_t bitIndex = offset % 8;

    // Check the offset.
    if (index >= _responseBufferLength - MODBUS_CRC_LENGTH)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    if (state)
    {
        bitSet(_responseBuffer[index], bitIndex);
    }
    else
    {
        bitClear(_responseBuffer[index], bitIndex);
    }

    return STATUS_OK;
}

/**
 * Escribe la entrada digital en el búfer de salida.
 *
 * @param offset El offset de la primera entrada en el búfer.
 * @param state El estado para escribir en el búfer (verdadero / falso).
 */
uint8_t Modbus::writeDiscreteInputToBuffer(int offset, bool state)
{
    return Modbus::writeCoilToBuffer(offset, state);
}

/**
 * Escribe el valor del registro en el búfer de salida.
 *
 * @param offset El offset desde el primer registro en el búfer.
 * @param value El valor de registro para escribir en el búfer.
 */
uint8_t Modbus::writeRegisterToBuffer(int offset, uint16_t value)
{
    // Verifique el código de la función.
    if (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_HOLDING_REGISTERS &&
        _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] != FC_READ_INPUT_REGISTERS)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    // (1 x valueBytes, n x values).
    uint16_t index = MODBUS_DATA_INDEX + 1 + (offset * 2);

    // Verifica el desplazamiento.
    if ((index + 2) > (_responseBufferLength - MODBUS_CRC_LENGTH))
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    _responseBuffer[index] = value >> 8;
    _responseBuffer[index + 1] = value & 0xFF;

    return STATUS_OK;
}

/**
 * Escribe una matriz uint8_t en el búfer de salida.
 *
 * @param offset El offset del primer registro de datos en el búfer de respuesta.
 * @param str La matriz para escribir en el búfer de respuesta.
 * @param length La longitud de la matriz.
 * @return STATUS_OK si tiene éxito, STATUS_ILLEGAL_DATA_ADDRESS si los datos no caben en el búfer.
 */
uint8_t Modbus::writeArrayToBuffer(int offset, uint16_t *str, uint8_t length)
{
    // Índice desde el que empezar a escribir (1 x valueBytes, n x values (offset)).
    uint8_t index = MODBUS_DATA_INDEX + 1 + (offset * 2);

    // Verifica si la matriz cabe en el espacio restante de la respuesta.
    if ((index + (length * 2)) > _responseBufferLength - MODBUS_CRC_LENGTH)
    {
        // Si no devuelve una excepción.
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }

    for (int i = 0; i < length; i++)
    {
        _responseBuffer[index + (i * 2)] = str[i] >> 8;
        _responseBuffer[index + (i * 2) + 1] = str[i] & 0xFF;
    }

    return STATUS_OK;
}

/**
 * ---------------------------------------------------
 *                  PRIVATE METHODS
 * ---------------------------------------------------
 */

/**
 * Devuelve verdadero si uno de los esclavos escucha la dirección dada.
 *
 * @param unitAddress La dirección recibida.
 */
bool Modbus::relevantAddress(uint8_t unitAddress)
{
    // Todos los dispositivos deben escuchar los mensajes de transmisión,
    // mantén la comprobacion local, ya que proporcionamos unitAddress
    if (unitAddress == MODBUS_BROADCAST_ADDRESS)
    {
        return true;
    }

    // Itere sobre todos los esclavos y compruebe si escucha la dirección dada, si es así, devuelva verdadero.
    for (uint8_t i = 0; i < _numberOfSlaves; ++i)
    {
        Serial.print("Direccion de un esclavo: ");
        Serial.print(_slaves[i].getUnitAddress(),HEX);
        if (_slaves[i].getUnitAddress() == unitAddress)
        {
            Serial.println(" ---Direccion Correcta");
            return true;
        }
    }

    return false;
}

/**
 * Ejecuta una devolución de llamada.
 *
 * @return El código de estado que representa el resultado de esta operación.
 */
uint8_t Modbus::executeCallback(uint8_t slaveAddress, uint8_t callbackIndex, uint16_t address, uint16_t length)
{
    // Busca el esclavo correcto para ejecutar la devolución de llamada.
    for (uint8_t i = 0; i < _numberOfSlaves; ++i)
    {
        ModbusCallback callback = _slaves[i].cbVector[callbackIndex];
        if (slaveAddress == MODBUS_BROADCAST_ADDRESS)
        {
            if (callback)
            {
                callback(Modbus::readFunctionCode(), address, length);
            }
        }
        else if (_slaves[i].getUnitAddress() == slaveAddress)
        {
            if (callback)
            {
                return callback(Modbus::readFunctionCode(), address, length);
            }
            else
            {
                return STATUS_ILLEGAL_FUNCTION;
            }
        }
    }
    // ¡No hay retorno en bucle para una transmisión, por lo tanto, regrese aquí sin error si es una transmisión!
    return slaveAddress == MODBUS_BROADCAST_ADDRESS ? STATUS_ACKNOWLEDGE : STATUS_ILLEGAL_FUNCTION;

}


/**
 * Calcule el CRC de la matriz de bytes pasada desde cero hasta la longitud pasada.
 *
 * @param buffer La matriz de bytes que contiene los datos.
 * @param length La longitud de la matriz de bytes.
 *
 * @return El CRC calculado como un entero de 16 bits sin signo.
 */
uint16_t Modbus::calculateCRC(uint8_t *buffer, int length)
{
    int i, j;
    uint16_t crc = 0xFFFF;
    uint16_t tmp;

    // Calculate the CRC.
    for (i = 0; i < length; i++)
    {
        crc = crc ^ buffer[i];

        for (j = 0; j < 8; j++)
        {
            tmp = crc & 0x0001;
            crc = crc >> 1;
            if (tmp)
            {
                crc = crc ^ 0xA001;
            }
        }
    }

    return crc;
}
