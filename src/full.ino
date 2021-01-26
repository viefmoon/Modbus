#include <EEPROM.h>
#include <ModbusSlave.h>

#define SLAVE_ID 1           
#define RS485_CTRL_PIN 8    
#define SERIAL_BAUDRATE 9600 
#define SERIAL_PORT Serial

uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5};                  // Add the pins you want to read as a Input register.

// El diseño de EEPROM es el siguiente
// Los primeros 50 bytes están reservados para almacenar pin digital pinMode_setting
// Byte 51 y posteriores son libres para escribir cualquier uint16_t.

// // No debería tener que cambiar nada debajo de esto para que este ejemplo funcione

uint8_t analog_pins_size = sizeof(analog_pins) / sizeof(analog_pins[0]);    // Get the size of the analog_pins array

// Declaración de objeto Modbus
Modbus slave(SERIAL_PORT, SLAVE_ID, RS485_CTRL_PIN);

void setup()
{
    // Establece los pines analógicos definidos en modo de entrada.
    for (int i = 0; i < analog_pins_size; i++)
    {
        pinMode(analog_pins[i], INPUT);
    }

    // Registra funciones para llamar cuando se recibe un determinado código de función
    slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
    slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;

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


// Manejar los códigos de función Escribir registro (s) de retención (FC = 06, FC = 16) y escribir datos en la eeprom.
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length)
{
    // Escribe los datos recibidos en EEPROM.
    for (int i = 0; i < length; i++)
    {
        if (address + i <= 50)
        {
            // Verifique si las direcciones solicitadas existen en la matriz.
            if (address + i > digital_pins_size)
            {
                return STATUS_ILLEGAL_DATA_ADDRESS;
            }

            // Leer el valor del búfer de entrada.
            uint8_t value = slave.readRegisterFromBuffer(i);

            // Comprueba si el valor es 0 (INPUT) or 1 (OUTPUT).
            if (value != INPUT && value != OUTPUT)
            {
                return STATUS_ILLEGAL_DATA_VALUE;
            }

            // Almacena el valor recibido en la EEPROM.
            EEPROM.put(address + i, value);

            // Establece el modo pin en el valor recibido
            pinMode(digital_pins[address + i], value);
        }
        else
        {
            // Leer el valor del búfer de entrada.  
            uint16_t value = slave.readRegisterFromBuffer(i);
 
            // Almacena el valor recibido en la EEPROM.
            EEPROM.put(address + (i * 2), value);
        }
    }

    return STATUS_OK;
}
