#include <string.h>
#include "ModbusSlave.h"

#define MODBUS_FRAME_SIZE 4
#define MODBUS_CRC_LENGTH 2

#define MODBUS_ADDRESS_INDEX 0
#define MODBUS_FUNCTION_CODE_INDEX 1
#define MODBUS_DATA_INDEX 2

#define MODBUS_BROADCAST_ADDRESS 1
#define MODBUS_ADDRESS_MIN 1
#define MODBUS_ADDRESS_MAX 247

#define MODBUS_HALF_SILENCE_MULTIPLIER 3
#define MODBUS_FULL_SILENCE_MULTIPLIER 7

#define readUInt16(arr, index) word(arr[index], arr[index + 1])
#define readCRC(arr, length) word(arr[(length - MODBUS_CRC_LENGTH) + 1], arr[length - MODBUS_CRC_LENGTH])

/**
 * Comprueba si tenemos una solicitud completa, analiza la solicitud, ejecuta la
 * correspondiente devolución de llamada registrada y escribe la respuesta.
 *
 * @return El número de bytes escritos como respuesta.
 */
uint8_t Modbus::poll()
{   
    // Si todavía estamos escribiendo un mensaje, déjelo terminar primero.
    if (_isResponseBufferWriting)
    {
        return Modbus::writeResponse();
    }

    // Espere un paquete de solicitud completo.
    if (!Modbus::readRequest())
    {
        return 0;
    }

    // Prepara el búfer de salida.
    memset(_responseBuffer, 0, MODBUS_MAX_BUFFER);
    _responseBuffer[MODBUS_ADDRESS_INDEX] = _requestBuffer[MODBUS_ADDRESS_INDEX];
    _responseBuffer[MODBUS_FUNCTION_CODE_INDEX] = _requestBuffer[MODBUS_FUNCTION_CODE_INDEX];
    _responseBufferLength = MODBUS_FRAME_SIZE;

    // Valida la solicitud entrante.
    if (!Modbus::validateRequest())
    {
        return 0;
    }

    // Ejecuta la solicitud entrante y crea la respuesta.
    uint8_t status = Modbus::createResponse();

    // Verifique si la ejecución de la devolución de llamada tuvo éxito.
    if (status != STATUS_OK)
    {
        return Modbus::reportException(status);
    }

    // Escribe la respuesta de creación en la interfaz serial.
    //Serial.println("Iniciando escritura serial...");
    return Modbus::writeResponse();
}

/**
 * Escribe el búfer de salida en serial stream.
 *
 * @return El número de bytes escritos.
 */
uint16_t Modbus::writeResponse()
{
    /**
     * Validar
     */

    // Compruebe si se ha creado una respuesta y si es la primera vez que se escribe.
    if (_responseBufferWriteIndex == 0 && _responseBufferLength >= MODBUS_FRAME_SIZE)
    {
        // Inicie la escritura.
        _isResponseBufferWriting = true;
    }

    // Si no estamos escribiendo o la dirección es la dirección de transmisión, limpieza y devolución.
    if (!_isResponseBufferWriting || isBroadcast())
    {
        _isResponseBufferWriting = false;
        _responseBufferWriteIndex = 0;
        _responseBufferLength = 0;
        return 0;
    }

    /**
     * Preparando
     */
    // Si esta es la primera escritura.
    if (_responseBufferWriteIndex == 0)
    {
        // Comprueba si ya pasamos 1.5T.
        if ((micros() - _lastCommunicationTime) <= (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER))
        {
            return 0;
        }

        // Calcular y añadir el CRC.
        uint16_t crc = Modbus::calculateCRC(_responseBuffer, _responseBufferLength - MODBUS_CRC_LENGTH);
        _responseBuffer[_responseBufferLength - MODBUS_CRC_LENGTH] = crc & 0xFF;
        _responseBuffer[(_responseBufferLength - MODBUS_CRC_LENGTH) + 1] = crc >> 8;

        // Inicie el modo de transmisión para RS485.
        if (_transmissionControlPin > MODBUS_CONTROL_PIN_NONE)
        {
            _serialStream.flush();
            digitalWrite(_transmissionControlPin, HIGH);
        }
    }
 
    /**
     * Transmitir
     */

    // Envía el búfer de salida a través del flujo serial.
    uint16_t length = 0;
    if (_serialTransmissionBufferLength > 0)
    {

        // Compruebe la longitud máxima de bytes que se enviarán en una llamada.
        uint16_t length = min(
            _serialStream.availableForWrite(),
            _responseBufferLength - _responseBufferWriteIndex);

        if (length > 0)
        { 
            // Escribe la longitud máxima de bytes en el flujo serial.
            length = _serialStream.write(
                _responseBuffer + _responseBufferWriteIndex,
                length);
            _responseBufferWriteIndex += length;
            _totalBytesSent += length;

        }

        // Comprueba si se han enviado todos los datos.
        if (_serialStream.availableForWrite() < _serialTransmissionBufferLength)
        {
            _lastCommunicationTime = micros();
            return length;
        }

        // Si el flujo serial informa que está vacío, asegúrese de que lo esté.
        // (`Serial` elimina bytes del búfer antes de enviarlos).
        _serialStream.flush();
    }
    else
    {  
        // Modo de compatibilidad para series de software mal escritas; aka AltSoftSerial.
        length = _responseBufferLength - _responseBufferWriteIndex;

        if (length > 0)
        {
            length = _serialStream.write(_responseBuffer, length);
            _serialStream.flush();
        }

        _responseBufferWriteIndex += length;
        _totalBytesSent += length;
    }

    // Si se han enviado todos los datos y han pasado más de 1,5T.
    if (_responseBufferWriteIndex >= _responseBufferLength && (micros() - _lastCommunicationTime) > (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER))
    {    
        // Finaliza la transmisión.
        if (_transmissionControlPin > MODBUS_CONTROL_PIN_NONE)
        {
            digitalWrite(_transmissionControlPin, LOW);
        }

        // Y limpia las variables.
        _isResponseBufferWriting = false;
        _responseBufferWriteIndex = 0;
        _responseBufferLength = 0;
    }


    return length;
}

/**
 * Lee una nueva solicitud del flujo en serie y llena el búfer de solicitudes.
 *
 * @return True si el búfer está lleno con una solicitud y está listo para ser procesado; de lo contrario falso.
 */
bool Modbus::readRequest()
{
    // Leer un paquete de datos e informar cuando se recibe por completo.
    uint16_t length = _serialStream.available();

    if (length > 0)
    {  
        // Si la lectura aún no ha comenzado.
        if (!_isRequestBufferReading)
        {   
        // Y ya se necesitaron 1.5T desde el último mensaje.
            if ((micros() - _lastCommunicationTime) > (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER))
            {
                // Inicie la lectura y borre el búfer.
                _requestBufferLength = 0;
                _isRequestBufferReading = true;
            }
            else
            {
                // Descartar los datos entrantes.
                _serialStream.read();
            }
        }

        // Si empezamos a leer.
        if (_isRequestBufferReading)
        {
            // Compruebe si el búfer no está ya lleno.
            if (_requestBufferLength == MODBUS_MAX_BUFFER)
            {
            // Y si es así, deja de leer.
                _isRequestBufferReading = false;
            }

            // Compruebe si hay suficiente espacio para los bytes entrantes en el búfer.
            length = min(length, MODBUS_MAX_BUFFER - _requestBufferLength);

            // Leer los datos del flujo serial en el búfer

            length = _serialStream.readBytes(_requestBuffer + _requestBufferLength, MODBUS_MAX_BUFFER - _requestBufferLength);

            // Si este es el primer ciclo de lectura, verifique la dirección para rechazar solicitudes irrelevantes.
            if (_requestBufferLength == 0 && length > MODBUS_ADDRESS_INDEX && !Modbus::relevantAddress(_requestBuffer[MODBUS_ADDRESS_INDEX]))
            {
                // Esta no es una de las direcciones de este dispositivo, deja de leer.
                _isRequestBufferReading = false;
            }

            // Mueve el puntero del búfer hacia adelante la cantidad de bytes leídos del flujo en serie.
            _requestBufferLength += length;
            _totalBytesReceived += length;
        }

        // Guarde la hora del último byte (s) recibido (s). 
        _lastCommunicationTime = micros();

        // Espere más datos.
        return false;
    }
    else
    {
        // Si todavía estamos leyendo pero no se han recibido datos para 1.5T, entonces este mensaje de solicitud está completo.
        if (_isRequestBufferReading && ((micros() - _lastCommunicationTime) > (_halfCharTimeInMicroSecond * MODBUS_HALF_SILENCE_MULTIPLIER)))
        {
            //Serial.print("Solicitud completa: ");
            // Detenga la lectura para permitir la lectura de nuevos mensajes.
            _isRequestBufferReading = false;
        }
        else
        {   
            // La solicitud aún no está completa, así que espere un poco más.
            return false;
        }
    }
                // Serial.print(_requestBuffer[0],HEX);
                // Serial.print(" ");
                // Serial.print(_requestBuffer[1],HEX);
                // Serial.print(" ");
                // Serial.print(_requestBuffer[2],HEX);
                // Serial.print(" ");
                // Serial.print(_requestBuffer[3],HEX);
                // Serial.print(" ");
                // Serial.print(_requestBuffer[4],HEX);
                // Serial.print(" ");
                // Serial.print(_requestBuffer[5],HEX);
                // Serial.print(" ");
                // Serial.print(_requestBuffer[6],HEX);
                // Serial.print(" ");
                // Serial.println(_requestBuffer[7],HEX);
                // Serial.println(" ");

    return !_isRequestBufferReading && (_requestBufferLength >= MODBUS_FRAME_SIZE);
}

/**
 * Valida el mensaje de solicitud actualmente en el búfer de entrada.
 *
 * @return True si la solicitud es válida; de lo contrario falso.
 */
bool Modbus::validateRequest()
{
    // Comprueba que el mensaje nos haya sido dirigido
    if (!Modbus::relevantAddress(_requestBuffer[MODBUS_ADDRESS_INDEX]))
    {
        return false;
    }
    // El tamaño mínimo del búfer (1 x Address, 1 x Function, n x Data, 2 x CRC).
    uint16_t expected_requestBufferSize = MODBUS_FRAME_SIZE;
    bool report_illegal_function=false;
    
    // Verifica la validez de los datos según el código de la función.
    //Serial.print("Codigo de funcion: ");
    Serial.print(_requestBuffer[MODBUS_FUNCTION_CODE_INDEX],DEC);
    switch (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX])
    {
        case FC_READ_EXCEPTION_STATUS:
        Serial.println(" -FC_READ_EXCEPTION_STATUS"); 
            // La transmisión no es compatible, así que ignore esta solicitud.
            if (_requestBuffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS)
            {
                return false;
            }
            break;
        case FC_READ_COILS:             // Read coils (digital read).
        Serial.println(" -FC_READ_COILS"); 
        break;
        case FC_READ_DISCRETE_INPUT:    // Read input state (digital read).
        Serial.println(" -FC_READ_DISCRETE_INPUT");   
        break;
        case FC_READ_HOLDING_REGISTERS: // Read holding registers (analog read).
        //Serial.println(" -FC_READ_HOLDING_REGISTERS");
        break;
        case FC_READ_INPUT_REGISTERS:{   // Read input registers (analog read).  
        Serial.println(" -FC_READ_INPUT_REGISTERS: ");  
            // La transmisión no es compatible, así que ignore esta solicitud.
            if (_requestBuffer[MODBUS_ADDRESS_INDEX] == MODBUS_BROADCAST_ADDRESS)
            {
                return false;
            }
            // Agregar bytes al tamaño de solicitud esperado (2 x Index, 2 x Count).
            expected_requestBufferSize += 4;
            }
            break;
        case FC_WRITE_COIL:     // Write coils (digital write).
        Serial.println(" -FC_WRITE_COIL"); 
        break;
        case FC_WRITE_REGISTER: // Write registers (digital write).
        Serial.println(" -FC_WRITE_REGISTER"); 
            // Agregar bytes al tamaño de solicitud esperado (2 x Index, 2 x Count).
            expected_requestBufferSize += 4;
            break;
        case FC_WRITE_MULTIPLE_COILS:
        Serial.println(" -FC_WRITE_MULTIPLE_COILS");
        break; 
        case FC_WRITE_MULTIPLE_REGISTERS:     
        Serial.println(" -FC_WRITE_MULTIPLE_REGISTERS"); 
            // Agregar bytes al tamaño de solicitud esperado (2 x Index, 2 x Count, 1 x Bytes).
            expected_requestBufferSize += 5;
            if (_requestBufferLength >= expected_requestBufferSize)
            {
            // Agregar bytes al tamaño de solicitud esperado (n x Bytes).
                expected_requestBufferSize += _requestBuffer[6];
            }
            break;
        default:       
            // Código de función desconocido.
             report_illegal_function=true;
    }

    // Si los datos recibidos son más pequeños de lo que esperamos, ignore esta solicitud.
    if (_requestBufferLength < expected_requestBufferSize)
    {
        Serial.print("Buffer recibido menor al esperado ");
        return false;
    }

    // Verifique la crc, y si no es correcta ignore la solicitud.
    uint16_t crc = readCRC(_requestBuffer, _requestBufferLength);
    // Serial.print("CRC leido: ");
    // Serial.println(crc,HEX);
    // Serial.print("CRC calculado: ");
    // Serial.println(Modbus::calculateCRC(_requestBuffer, _requestBufferLength - MODBUS_CRC_LENGTH),HEX);

    if (Modbus::calculateCRC(_requestBuffer, _requestBufferLength - MODBUS_CRC_LENGTH) != crc)
    {
        Serial.println("CRC incorrecto");
        return false;
    }
    
    // Verifique la crc, y si no es correcta ignore la solicitud.
    if (report_illegal_function)
    {
        Modbus::reportException(STATUS_ILLEGAL_FUNCTION);
        return false;
    }
     
    // Establezca la longitud a leer de la solicitud a la longitud esperada calculada.
    _requestBufferLength = expected_requestBufferSize;
    return true;
}

/**
 * Llena el búfer de salida con la respuesta a la solicitud del búfer de entrada.
 *
 * @return El código de estado que representa el resultado de esta operación.
 */
uint8_t Modbus::createResponse()
{
    uint16_t firstAddress;
    uint16_t addressesLength;
    uint8_t callbackIndex;
    uint16_t requestUnitAddress = _requestBuffer[MODBUS_ADDRESS_INDEX];

    //Serial.print("FUNCION DE DEVOLUCION: ");
    // Haga coincidir el código de la función con una devolución de llamada y ejecútelo y prepare el búfer de respuesta.
    switch (_requestBuffer[MODBUS_FUNCTION_CODE_INDEX])
    {
    case FC_READ_EXCEPTION_STATUS:        
    Serial.println("FC_READ_EXCEPTION_STATUS ");
    // Rechazar solicitudes de lectura de difusión
        if (requestUnitAddress == MODBUS_BROADCAST_ADDRESS)
        {
            return STATUS_ILLEGAL_FUNCTION;
        }
       
    // Suma la longitud de los datos de respuesta a la longitud de la salida.
        _responseBufferLength += 1;
   
        // Ejecuta la devolución de llamada y devuelve el código de estado.
        return Modbus::executeCallback(requestUnitAddress, CB_READ_EXCEPTION_STATUS, 0, 8);
        break;

    case FC_READ_COILS:          // Read coils (digital out state).
    Serial.println("FC_READ_COILS ");
    break;
    case FC_READ_DISCRETE_INPUT: // Read input state (digital in). 
    Serial.println("FC_READ_DISCRETE_INPUT ");     
    break;

    case FC_READ_HOLDING_REGISTERS: // Read holding registers (analog out state)
    //Serial.println("FC_READ_HOLDING_REGISTERS ");
        // Rechazar solicitudes de lectura de difusión
        if (requestUnitAddress == MODBUS_BROADCAST_ADDRESS)
        {
            return STATUS_ILLEGAL_FUNCTION;
        }

        // Leer la primera dirección y el número de entradas.
        firstAddress = readUInt16(_requestBuffer, MODBUS_DATA_INDEX);
        // //Serial.print("Primera direccion de registro: ");
        // //Serial.println(firstAddress,HEX);
        addressesLength = readUInt16(_requestBuffer, MODBUS_DATA_INDEX + 2);
        // Serial.print("Longitud de direcciones: ");
        // Serial.println(addressesLength);

        // Calcula la longitud de los datos de respuesta y agrégala a la longitud del búfer de salida.
        _responseBuffer[MODBUS_DATA_INDEX] = 2 * addressesLength;
        _responseBufferLength += 1 + _responseBuffer[MODBUS_DATA_INDEX];
        // Serial.print("Longitud de respuesta de buffer: ");
        // Serial.println(_responseBufferLength);
      
        // Ejecuta la devolución de llamada y devuelve el código de estado.
        callbackIndex = _requestBuffer[MODBUS_FUNCTION_CODE_INDEX] == FC_READ_HOLDING_REGISTERS ? CB_READ_HOLDING_REGISTERS : CB_READ_INPUT_REGISTERS;
        return Modbus::executeCallback(requestUnitAddress, callbackIndex, firstAddress, addressesLength);
        break;
    case FC_READ_INPUT_REGISTERS:   // Read input registers (analog in)  
    Serial.println("FC_READ_INPUT_REGISTERS ");
    break;

    case FC_WRITE_COIL: // Write one coil (digital out).       
    Serial.println("FC_WRITE_COIL ");
    break;

    case FC_WRITE_REGISTER: // Write one holding register (analog out).
    Serial.println("FC_WRITE_REGISTER ");
    break;

    case FC_WRITE_MULTIPLE_COILS: // Write multiple coils (digital out)  
    Serial.println("FC_WRITE_MULTIPLE_COILS ");     
    break;

    case FC_WRITE_MULTIPLE_REGISTERS: // Write multiple holding registers (analog out). 
    break;

    default:
        return STATUS_ILLEGAL_FUNCTION;
    }
}

/**
 * Llena el búfer de salida con una excepción basada en la solicitud en el búfer de entrada y escribe la respuesta en el flujo en serie.
 *
 * @param exceptionCode El código de estado para informar.
 * @return El número de bytes escritos.
 */
uint16_t Modbus::reportException(uint8_t exceptionCode)
{

    // La transmisión no es compatible, así que ignore esta solicitud.
    if (isBroadcast())
    {
        return 0;
    }

    // Agrega exceptionCode al búfer de salida.
    _responseBufferLength = MODBUS_FRAME_SIZE + 1;
    _responseBuffer[MODBUS_FUNCTION_CODE_INDEX] |= 0x80;
    _responseBuffer[MODBUS_DATA_INDEX] = exceptionCode;

    return Modbus::writeResponse();
}

/**
 * Devuelve un valor booleano que indica si la solicitud se está procesando actualmente
 * es un mensaje de difusión y, por lo tanto, no necesita respuesta.
 *
 * @return True si el mensaje de solicitud actual es un mensaje de difusión; de lo contrario falso.
 */
bool Modbus::isBroadcast()
{
    return Modbus::readUnitAddress() == MODBUS_BROADCAST_ADDRESS;
}