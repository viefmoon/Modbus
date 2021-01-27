#include <EEPROM.h>
#include <ModbusSlave.h>

#define SLAVE_ID 0x11          
#define RS485_CTRL_PIN 8    
#define SERIAL_BAUDRATE 9600 
#define SERIAL_PORT Serial

// El diseño de EEPROM es el siguiente
// Los primeros 50 bytes están reservados para almacenar pin digital pinMode_setting
// Byte 51 y posteriores son libres para escribir cualquier uint16_t.

// // No debería tener que cambiar nada debajo de esto para que este ejemplo funcione

// Declaración de objeto Modbus
Modbus slave(SERIAL_PORT, SLAVE_ID, RS485_CTRL_PIN);

void setup()
{
    // Registra funciones para llamar cuando se recibe un determinado código de función
    slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;

    // Establezca el puerto serie y el esclavo a la velocidad de transmisión dada.
    SERIAL_PORT.begin(SERIAL_BAUDRATE);
    slave.begin(SERIAL_BAUDRATE);
}

void loop()
{
    // Escuche las solicitudes de Modbus en el puerto serie.
    // Cuando se recibe una solicitud, se validará.
    // Y si hay una función registrada en el código de función recibido, esta función se ejecutará.
    slave.poll();
}

// Funciones del manejador Modbus
// Las funciones del controlador deben devolver un uint8_t y tomar los siguientes parámetros:
// uint8_t fc - código de función
// address uint16_t - primer registro / dirección de bobina
// uint16_t length / status - longitud de los datos / estado de la bobina

// Manejar el código de función Read Holding Registers (FC = 03) y volver a escribir los valores de EEPROM (Holding Registers).
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length)
{
// Leer los registros EEPROM solicitados.
    for (int i = 0; i < length; i++)
    {
        // Por debajo de 50 está reservado para pinModes, por encima de 50 es de uso gratuito.
        if (address + i <= 50)
        {
            uint8_t value;

            // Leer un valor de la EEPROM.
            EEPROM.get((address + i), value);

            // Escribe el pinMode de EEPROM en el búfer de respuesta.
            slave.writeRegisterToBuffer(i, value);
        }
        else
        {
            uint16_t value;

            
            // Leer un valor de la EEPROM.
            EEPROM.get(address + (i * 2), value);
            
            // Escribe el valor de EEPROM en el búfer de respuesta.
            slave.writeRegisterToBuffer(i, value);
        }
    }

    return STATUS_OK;
}

